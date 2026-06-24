/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_TAG.c
  * @brief          : DW1000 Diagnostic Asymmetric DS-TWR - TAG
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "dw1000.h"
#include "dw1000_stm32.h"
#include "dw1000_time.h"
#include <string.h>
#include <stdio.h>

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

static void DW1000_HardReset(dw1000_HandleTypeDef *dw1000);
static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw1000);
static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw1000);
static void UART_Print(const char *str);

/* DIAGNOSTIC & RECOVERY FUNCTIONS */
static void DW1000_ClearAllStatus_Manual(dw1000_HandleTypeDef *dw1000);
static void DW1000_Force_TRXOFF(dw1000_HandleTypeDef *dw1000);
static void DW1000_DisableFrameFiltering(dw1000_HandleTypeDef *dw1000);
static void DW1000_LoadLDE_Microcode(dw1000_HandleTypeDef *dw1000);

#define FC_POLL       0x20
#define FC_RESPONSE   0x21
#define FC_FINAL      0x22
#define FC_REPORT     0x23 // New message type for DS-TWR

#define DW1000_SS_PIN   GPIO_PIN_0
#define DW1000_SS_PORT  GPIOB
#define DW1000_RST_PIN  GPIO_PIN_8
#define DW1000_RST_PORT GPIOA
#define DW1000_IRQ_PIN  GPIO_PIN_9
#define DW1000_IRQ_PORT GPIOA

static dw1000_HandleTypeDef dw1000;

int main(void)
{
  uint8_t rx_buf[128]; 
  uint8_t tx_buf[128];
  char msg[250];

  dw1000_timestamp_t t1 = 0; // POLL TX
  dw1000_timestamp_t t4 = 0; // RESP RX
  dw1000_timestamp_t t5 = 0; // FINAL TX

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  dw1000.spi    = &hspi1;
  dw1000.ss_port = DW1000_SS_PORT;
  dw1000.ss_pin  = DW1000_SS_PIN;

  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);
  DW1000_HardReset(&dw1000);

  /* 1. Master Init */
  dw1000_Init(&dw1000, 5, DW1000_PRF_16MHZ, DW1000_TX_FCTRL_TXBR_110K);

  /* 2. Configure Addresses */
  dw1000_pan_addr_t pan_addr;
  pan_addr.pan_id    = 0xDECA;
  pan_addr.short_addr = 0x0002;
  dw1000_SetPanAddress(&dw1000, &pan_addr);

  /* 3. Apply Hardware Fixes */
  DW1000_LoadLDE_Microcode(&dw1000); 
  DW1000_Force_TRXOFF(&dw1000);
  DW1000_DisableFrameFiltering(&dw1000);
  DW1000_ClearAllStatus_Manual(&dw1000);

  UART_Print("\r\n========================================\r\n");
  UART_Print("[TAG] Asymmetric DS-TWR Initialized.\r\n");
  UART_Print("========================================\r\n");

  while (1)
  {
    UART_Print("\r\n[TAG] --- NEW CYCLE ---\r\n");
    
    /* ---- 1. Send POLL --------------------------- */
    DW1000_Force_TRXOFF(&dw1000); 
    DW1000_ClearAllStatus_Manual(&dw1000);
    
    tx_buf[0] = FC_POLL;
    for(int i=1; i<16; i++) tx_buf[i] = 0x00; 
    
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 16, 1);
    dw1000_StartTransmit(&dw1000, 16, 1); 

    uint32_t tx_start = HAL_GetTick();
    uint8_t sent = 0;
    while (HAL_GetTick() - tx_start < 50) {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);
      if ((sys_status[0] >> 7) & 1) { sent = 1; break; }
    }

    if (!sent) {
      UART_Print("[TAG] TX POLL timeout. Hardware hung?\r\n");
      DW1000_Force_TRXOFF(&dw1000); 
      HAL_Delay(200);
      continue;
    }
    t1 = dw1000_GetTxTimestamp(&dw1000);

    /* ---- 2. Wait for RESPONSE ----------------- */
    DW1000_Force_TRXOFF(&dw1000); 
    DW1000_ClearAllStatus_Manual(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    uint32_t rx_start = HAL_GetTick();
    uint8_t rx_success = 0;
    
    while (HAL_GetTick() - rx_start < 100) {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);
      
      if (((sys_status[1] >> 5) & 1) && ((sys_status[1] >> 6) & 1)) {
        rx_success = 1;
        break;
      }
      if (((sys_status[1] >> 7) & 1) || ((sys_status[4] >> 1) & 1)) {
        break; // Break on FCS error or Preamble Rejection
      }
    }

    if (!rx_success) {
      UART_Print("[TAG] RX RESPONSE timeout/error.\r\n");
      DW1000_Force_TRXOFF(&dw1000); 
      HAL_Delay(200);
      continue;
    }

    uint16_t len = dw1000_GetDataReceivedLength(&dw1000, 1);
    dw1000_GetDataReceived(&dw1000, rx_buf, (len > 128) ? 128 : len);

    if (rx_buf[0] != FC_RESPONSE) continue;
    t4 = dw1000_GetRxTimestamp(&dw1000);

    /* ---- 3. Send FINAL -------------------------- */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);
    
    tx_buf[0] = FC_FINAL;
    for(int i=1; i<16; i++) tx_buf[i] = 0x00;
    
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 16, 1);
    dw1000_StartTransmit(&dw1000, 16, 1); 

    tx_start = HAL_GetTick();
    sent = 0;
    while (HAL_GetTick() - tx_start < 50) {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);
      if ((sys_status[0] >> 7) & 1) { sent = 1; break; }
    }

    if (!sent) {
      DW1000_Force_TRXOFF(&dw1000);
      HAL_Delay(200);
      continue;
    }
    t5 = dw1000_GetTxTimestamp(&dw1000);

    /* ---- 4. Send REPORT (Payload with times) ---- */
    HAL_Delay(5); // Give Anchor 5ms to transition to RX state
    
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);

    dw1000_timestamp_t t_round1 = (t4 - t1) & 0xFFFFFFFFFFULL;
    dw1000_timestamp_t t_reply2 = (t5 - t4) & 0xFFFFFFFFFFULL;

    tx_buf[0] = FC_REPORT;
    for (int b = 0; b < 5; b++) tx_buf[1 + b] = (uint8_t)(t_round1 >> (8 * b));
    for (int b = 0; b < 5; b++) tx_buf[6 + b] = (uint8_t)(t_reply2 >> (8 * b));
    for (int i = 11; i < 16; i++) tx_buf[i] = 0x00;

    dw1000_SetDataToTransmit(&dw1000, tx_buf, 16, 1);
    dw1000_StartTransmit(&dw1000, 16, 1); 

    tx_start = HAL_GetTick();
    while (HAL_GetTick() - tx_start < 50) {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);
      if ((sys_status[0] >> 7) & 1) break;
    }

    UART_Print("[TAG] Cycle Success. Pausing...\r\n");
    HAL_Delay(250);
  }
}

/* --- HARDWARE FIX FUNCTIONS --- */
static void DW1000_LoadLDE_Microcode(dw1000_HandleTypeDef *dw1000) {
    uint8_t pmsc_ctrl0[2] = {0x01, 0x03}; 
    dw1000_WriteSubData(dw1000, 0x36, 0x00, pmsc_ctrl0, 2);
    uint8_t otp_ctrl[2] = {0x00, 0x80};   
    dw1000_WriteSubData(dw1000, 0x2D, 0x06, otp_ctrl, 2);
    HAL_Delay(2);                         
    uint8_t pmsc_ctrl0_restore[2] = {0x00, 0x02}; 
    dw1000_WriteSubData(dw1000, 0x36, 0x00, pmsc_ctrl0_restore, 2);
}

static void DW1000_Force_TRXOFF(dw1000_HandleTypeDef *dw1000) {
    uint8_t sys_ctrl[4] = {0x40, 0x00, 0x00, 0x00}; 
    dw1000_WriteData(dw1000, 0x0D, sys_ctrl, 4); 
    HAL_Delay(1); 
}

static void DW1000_ClearAllStatus_Manual(dw1000_HandleTypeDef *dw1000) {
  uint8_t clear_status[5] = {0xFC, 0xFF, 0xFF, 0xFF, 0xFF}; 
  dw1000_WriteData(dw1000, 0x0F, clear_status, 5);
}

static void DW1000_DisableFrameFiltering(dw1000_HandleTypeDef *dw1000) {
    uint8_t sys_cfg[4];
    dw1000_ReadData(dw1000, 0x04, sys_cfg, 4); 
    sys_cfg[0] &= ~0x01; 
    dw1000_WriteData(dw1000, 0x04, sys_cfg, 4);
}

static void UART_Print(const char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void DW1000_HardReset(dw1000_HandleTypeDef *dw1000) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DW1000_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW1000_RST_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5); 
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_RST_PORT, &GPIO_InitStruct);
  HAL_Delay(15); 
}

static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[5];
  dw1000_ReadData(dw1000, DW1000_TX_TIME, buffer, 5);
  return ((dw1000_timestamp_t)buffer[4] << 32) | ((dw1000_timestamp_t)buffer[3] << 24) |
         ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] << 8)  |
          (dw1000_timestamp_t)buffer[0];
}

static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw1000) {
  uint8_t buffer[5];
  dw1000_ReadData(dw1000, DW1000_RX_TIME, buffer, 5);
  return ((dw1000_timestamp_t)buffer[4] << 32) | ((dw1000_timestamp_t)buffer[3] << 24) |
         ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] << 8)  |
          (dw1000_timestamp_t)buffer[0];
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  HAL_SPI_Init(&hspi1);
}

static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1) { } }