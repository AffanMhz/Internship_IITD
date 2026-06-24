/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_DW1000Ranging_TAG.c
  * @brief          : DW1000 Simple Two-Way Ranging  -  TAG role
  *                   NUCLEO-G070RB
  ******************************************************************************
  * Same pin map, same sequencing notes as ANCHOR.
  * See ANCHOR file for full commentary.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "dw1000.h"
#include "dw1000_stm32.h"
#include "dw1000_time.h"
#include <string.h>
#include <stdio.h>

SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init_Slow(void);
static void MX_SPI1_Init_Fast(void);
static void MX_USART2_UART_Init(void);

static void DW1000_HardReset(void);
static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw);
static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw);
static void UART_Print(const char *str);

#define FC_POLL       0x20u
#define FC_RESPONSE   0x21u
#define FC_FINAL      0x22u

#define DW1000_SS_PIN   GPIO_PIN_0
#define DW1000_SS_PORT  GPIOB
#define DW1000_RST_PIN  GPIO_PIN_8
#define DW1000_RST_PORT GPIOA
#define DW1000_IRQ_PIN  GPIO_PIN_9
#define DW1000_IRQ_PORT GPIOA

#define TIMEOUT_CPLOCK_MS    200u
#define TIMEOUT_TX_MS         50u
#define TIMEOUT_RX_RESP_MS   200u   /* anchor must reply within ~200 ms */

static dw1000_HandleTypeDef dw1000;

/* ========================================================================== */
int main(void)
{
  uint8_t  rx_buf[16];
  uint8_t  tx_buf[16];
  char     msg[200];
  uint8_t  sys_status[DW1000_SYS_STATUS_LEN];

  dw1000_timestamp_t t_poll_tx  = 0;
  dw1000_timestamp_t t_resp_rx  = 0;
  dw1000_timestamp_t t_final_tx = 0;
  dw1000_timestamp_t t_round    = 0;   /* piggy-backed to next FINAL */

  uint8_t  frame_received;
  uint8_t  sent;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  DW1000_HardReset();
  MX_SPI1_Init_Slow();   /* 2 MHz until CPLOCK */

  dw1000.spi    = &hspi1;
  dw1000.ss_port = DW1000_SS_PORT;
  dw1000.ss_pin  = DW1000_SS_PIN;
  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);

  /* Device ID check */
  dw1000_dev_id_t dev_id = dw1000_GetDevID(&dw1000);
  sprintf(msg, "\r\n[TAG] DEV_ID model=0x%02X ver=%u rev=%u ridtag=0x%04X %s\r\n",
          dev_id.model, dev_id.ver, dev_id.rev, dev_id.ridtag,
          (dev_id.ridtag == 0xDECA) ? "(OK)" : "(WRONG - SPI problem?)");
  UART_Print(msg);

  /* PAN / channel config */
  dw1000_pan_addr_t pan_addr = { .pan_id = 0xDECA, .short_addr = 0x0002 };
  dw1000_SetPanAddress(&dw1000, &pan_addr);

  dw1000_channel_t chan = {
    .tx_chan = 5, .rx_chan = 5,
    .prf = DW1000_PRF_16MHZ,
    .tx_preamble = 0x4,
    .rx_preamble = 0x4
  };
  dw1000_SetChannel(&dw1000, &chan);

  /* Full tuning init */
  uint8_t init_ok = dw1000_Init(&dw1000, 5, DW1000_PRF_16MHZ,
                                 DW1000_TX_FCTRL_TXBR_110K);
  sprintf(msg, "[TAG] dw1000_Init() %s\r\n",
          init_ok ? "OK" : "FAILED (sub-reg readback mismatch)");
  UART_Print(msg);

  uint16_t agc1 = dw1000_ReadSubReg16(&dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB);
  uint8_t  lde1 = dw1000_ReadSubReg8 (&dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG1_SUB);
  uint32_t drx2 = dw1000_ReadSubReg32(&dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE2_SUB);
  sprintf(msg, "[TAG] AGC_TUNE1=0x%04X(exp 0x8870) LDE_CFG1=0x%02X(exp 0x6D)"
               " DRX_TUNE2=0x%08lX(exp 0x311A002D)\r\n",
          agc1, lde1, (unsigned long)drx2);
  UART_Print(msg);

  /* Wait for RF PLL lock — MANDATORY */
  UART_Print("[TAG] Waiting for CPLOCK...\r\n");
  uint8_t cplock_ok = dw1000_WaitForCPLOCK(&dw1000, TIMEOUT_CPLOCK_MS);
  dw1000_GetSystemStatus(&dw1000, sys_status);
  sprintf(msg, "[TAG] CPLOCK %s. SYS_STATUS[0..4]=%02X %02X %02X %02X %02X\r\n",
          cplock_ok ? "LOCKED" : "TIMEOUT (RF dead - check power/wiring)",
          sys_status[0], sys_status[1], sys_status[2], sys_status[3], sys_status[4]);
  UART_Print(msg);

  if (!cplock_ok) {
    UART_Print("[TAG] FATAL: CPLOCK never asserted. Halting.\r\n");
    while (1) { HAL_Delay(1000); UART_Print("[TAG] Still no CPLOCK.\r\n"); }
  }

  /* Switch to fast SPI */
  MX_SPI1_Init_Fast();
  UART_Print("[TAG] SPI switched to 8 MHz (PLL locked).\r\n");

  dw1000_pan_addr_t pan_rb  = dw1000_GetPanAddress(&dw1000);
  dw1000_channel_t  chan_rb = dw1000_GetChannel(&dw1000);
  sprintf(msg, "[TAG] PAN=0x%04X ADDR=0x%04X CH tx=%u rx=%u PRF=%u\r\n",
          pan_rb.pan_id, pan_rb.short_addr,
          chan_rb.tx_chan, chan_rb.rx_chan, chan_rb.prf);
  UART_Print(msg);

  dw1000_ClearAllStatus(&dw1000);
  UART_Print("[TAG] Ready. Starting ranging cycles...\r\n");

  /* ==========================================================================
   * Main ranging loop
   * ========================================================================== */
  while (1)
  {
    /* ---- 1. Send POLL --------------------------------------------------- */
    dw1000_IdleMode(&dw1000);
    HAL_Delay(1);
    dw1000_ClearTransmitStatus(&dw1000);
    tx_buf[0] = FC_POLL;
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 1, 1);
    dw1000_StartTransmit(&dw1000, 1, 1);

    sent = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TIMEOUT_TX_MS) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      if (dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXFRS)) {
        sent = 1; break;
      }
    }
    if (!sent) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      uint32_t state = dw1000_GetSystemState(&dw1000);
      sprintf(msg, "[TAG] POLL TX timeout. STATUS=%02X%02X%02X%02X%02X "
                   "STATE=0x%06lX TXBERR=%u\r\n",
              sys_status[4],sys_status[3],sys_status[2],sys_status[1],sys_status[0],
              (unsigned long)state,
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXBERR));
      UART_Print(msg);
      HAL_Delay(200);
      continue;
    }

    t_poll_tx = dw1000_GetTxTimestamp(&dw1000);
    dw1000_ClearTransmitStatus(&dw1000);
    sprintf(msg, "[TAG] POLL sent. t_poll_tx=%llu\r\n", (unsigned long long)t_poll_tx);
    UART_Print(msg);

    /* ---- 2. Receive RESPONSE -------------------------------------------- */
    dw1000_IdleMode(&dw1000);
    HAL_Delay(1);
    dw1000_ClearReceiveStatus(&dw1000);
    dw1000_ClearAllStatus(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    frame_received = 0;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TIMEOUT_RX_RESP_MS) {
      if (dw1000_ReceiveDataFrameReady(&dw1000)) {
        frame_received = 1; break;
      }
    }
    if (!frame_received) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      uint32_t state = dw1000_GetSystemState(&dw1000);
      sprintf(msg, "[TAG] RESPONSE timeout. STATUS=%02X%02X%02X%02X%02X "
                   "STATE=0x%06lX CPLOCK=%u RXPRD=%u RXSFDD=%u "
                   "RXPHE=%u RXFCE=%u RXRFSL=%u LDEERR=%u RXSFDTO=%u RXFCG=%u\r\n",
              sys_status[4],sys_status[3],sys_status[2],sys_status[1],sys_status[0],
              (unsigned long)state,
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_CPLOCK),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXPRD),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXSFDD),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXPHE),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXFCE),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXRFSL),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_LDEERR),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXSFDTO),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXFCG));
      UART_Print(msg);
      HAL_Delay(200);
      continue;
    }

    uint16_t len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len == 0 || len > sizeof(rx_buf)) {
      sprintf(msg, "[TAG] RESPONSE bad len=%u\r\n", len);
      UART_Print(msg);
      dw1000_ClearReceiveStatus(&dw1000);
      HAL_Delay(200);
      continue;
    }
    dw1000_GetDataReceived(&dw1000, rx_buf, len);
    dw1000_ClearReceiveStatus(&dw1000);

    sprintf(msg, "[TAG] RX ok len=%u fc=0x%02X\r\n", len, rx_buf[0]);
    UART_Print(msg);

    if (rx_buf[0] != FC_RESPONSE) {
      UART_Print("[TAG] Not RESPONSE, ignoring.\r\n");
      HAL_Delay(200);
      continue;
    }

    t_resp_rx = dw1000_GetRxTimestamp(&dw1000);
    sprintf(msg, "[TAG] RESPONSE received. t_resp_rx=%llu\r\n",
            (unsigned long long)t_resp_rx);
    UART_Print(msg);

    /* ---- 3. Send FINAL (carries t_round from PREVIOUS cycle) ------------ */
    dw1000_IdleMode(&dw1000);
    HAL_Delay(1);
    dw1000_ClearTransmitStatus(&dw1000);
    tx_buf[0] = FC_FINAL;
    for (int b = 0; b < 5; b++)
      tx_buf[1 + b] = (uint8_t)(t_round >> (8 * b));
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 6, 1);
    dw1000_StartTransmit(&dw1000, 6, 1);

    sent = 0;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TIMEOUT_TX_MS) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      if (dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXFRS)) {
        sent = 1; break;
      }
    }
    if (!sent) {
      UART_Print("[TAG] FINAL TX timeout.\r\n");
      HAL_Delay(200);
      continue;
    }

    t_final_tx = dw1000_GetTxTimestamp(&dw1000);
    dw1000_ClearTransmitStatus(&dw1000);

    /* Update t_round for THIS cycle — will be sent in NEXT FINAL */
    t_round = t_final_tx - t_poll_tx;
    (void)t_resp_rx;

    sprintf(msg, "[TAG] FINAL sent. t_final_tx=%llu t_round(next)=%llu (%.4f us)\r\n",
            (unsigned long long)t_final_tx,
            (unsigned long long)t_round,
            DW1000_Time_TimestampToMicroseconds(t_round));
    UART_Print(msg);

    HAL_Delay(200);
  }
}

/* ============================================================================
 * Helper implementations  (identical to ANCHOR)
 * ========================================================================== */
static void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void DW1000_HardReset(void)
{
  GPIO_InitTypeDef g = {0};
  g.Pin   = DW1000_RST_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW1000_RST_PORT, &g);
  HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  g.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_RST_PORT, &g);
  HAL_Delay(10);
}

static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw)
{
  uint8_t buf[5];
  dw1000_ReadData(dw, DW1000_TX_TIME, buf, 5);
  return ((dw1000_timestamp_t)buf[4]<<32)|((dw1000_timestamp_t)buf[3]<<24)|
         ((dw1000_timestamp_t)buf[2]<<16)|((dw1000_timestamp_t)buf[1]<<8) |
          (dw1000_timestamp_t)buf[0];
}

static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw)
{
  uint8_t buf[5];
  dw1000_ReadData(dw, DW1000_RX_TIME, buf, 5);
  return ((dw1000_timestamp_t)buf[4]<<32)|((dw1000_timestamp_t)buf[3]<<24)|
         ((dw1000_timestamp_t)buf[2]<<16)|((dw1000_timestamp_t)buf[1]<<8) |
          (dw1000_timestamp_t)buf[0];
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSIDiv              = RCC_HSI_DIV1;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState        = RCC_PLL_ON;
  osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM            = RCC_PLLM_DIV1;
  osc.PLL.PLLN            = 8;
  osc.PLL.PLLP            = RCC_PLLP_DIV2;
  osc.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init_Slow(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; /* 2 MHz */
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7;
  hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init_Fast(void)
{
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 8 MHz */
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = 115200;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2)                                           != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart2)                               != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);
  g.Pin = DW1000_SS_PIN; g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW1000_SS_PORT, &g);
  g.Pin = DW1000_RST_PIN; g.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_RST_PORT, &g);
  g.Pin = DW1000_IRQ_PIN; g.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_IRQ_PORT, &g);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
