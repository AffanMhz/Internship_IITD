/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_TAG.c
  * @brief          : DW1000 DS-TWR TAG — compact serial output
  *
  * Serial output format:
  *   Successful FINAL sent: "T:ok,r1=<round1>\n"
  *   Errors (rare):         "E:<code>\n"
  *   HPDWARN (increase FINAL_TX_DELAY_US if this appears): "E:hpdwarn\n"
  *
  * KEY TIMING RULE — no UART between T4 capture and FINAL TX setup:
  *   The UART_Print after "RESPONSE received" in the original code added
  *   ~4ms of blocking HAL_UART_Transmit *after* T4 but *before*
  *   dw1000_PlanDelayedTransmit. While T4 itself is already hardware-latched
  *   and safe, that gap was eating into the FINAL_TX_DELAY_US budget.
  *   With verbose prints removed, FINAL_TX_DELAY_US=3000us is generous.
  *   If HPDWARN appears, increase it; do not re-add UART prints there.
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
DMA_HandleTypeDef hdma_usart2_tx;

void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

static void DW1000_HardReset(dw1000_HandleTypeDef *dw1000);
static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw1000);
static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw1000);
static void UART_Print(const char *str);

static void DW1000_ClearAllStatus_Manual(dw1000_HandleTypeDef *dw1000);
static void DW1000_Force_TRXOFF(dw1000_HandleTypeDef *dw1000);
static void DW1000_DisableFrameFiltering(dw1000_HandleTypeDef *dw1000);
static void DW1000_LoadLDE_Microcode(dw1000_HandleTypeDef *dw1000);
static void DW1000_EnableEventIrq(dw1000_HandleTypeDef *dw1000);
static uint8_t DW1000_WaitForIrqStatus(uint32_t timeout_ms, uint8_t *sys_status);
static void TAG_RetryBackoff(void);

#define FC_POLL       0x20
#define FC_RESPONSE   0x21
#define FC_FINAL      0x22

/* Must comfortably exceed SPI+software overhead between
 * dw1000_PlanDelayedTransmit() and the TXSTRT bit being written.
 * With UART prints removed from that path, 3ms is generous.
 * If E:hpdwarn appears, increase this value. */
#define FINAL_TX_DELAY_US  3000

/* Keep failed cycles from hammering the radio continuously, but avoid the
 * original 200 ms sleep that limited successful ranging to about 5 Hz. */
#define TAG_ERROR_BACKOFF_MS  2

#define DW1000_SS_PIN   GPIO_PIN_0
#define DW1000_SS_PORT  GPIOB
#define DW1000_RST_PIN  GPIO_PIN_8
#define DW1000_RST_PORT GPIOA
#define DW1000_IRQ_PIN  GPIO_PIN_9
#define DW1000_IRQ_PORT GPIOA

static dw1000_HandleTypeDef dw1000;
static volatile uint8_t dw1000_irq_pending = 0;
static volatile uint8_t uart_tx_busy = 0;
static uint8_t uart_tx_dma_buf[96];

int main(void)
{
  uint8_t rx_buf[128];
  uint8_t tx_buf[128];
  char    msg[80];

  dw1000_timestamp_t t_poll_tx  = 0;
  dw1000_timestamp_t t_resp_rx  = 0;

  HAL_Init();
  SystemClock_Config();
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  dw1000.spi     = &hspi1;
  dw1000.ss_port = DW1000_SS_PORT;
  dw1000.ss_pin  = DW1000_SS_PIN;

  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);
  DW1000_HardReset(&dw1000);

  dw1000_Init(&dw1000, 5, DW1000_PRF_16MHZ, DW1000_TX_FCTRL_TXBR_6M8);

  dw1000_pan_addr_t pan_addr;
  pan_addr.pan_id     = 0xDECA;
  pan_addr.short_addr = 0x0002;
  dw1000_SetPanAddress(&dw1000, &pan_addr);

  DW1000_LoadLDE_Microcode(&dw1000);
  DW1000_Force_TRXOFF(&dw1000);
  DW1000_DisableFrameFiltering(&dw1000);
  DW1000_EnableEventIrq(&dw1000);
  DW1000_ClearAllStatus_Manual(&dw1000);

  UART_Print("TAG DS-TWR ready. Format: T:ok,r1=<ticks> E:<code>\r\n");

  while (1)
  {
    /* ---- 1. Send POLL (T1) ---------------------------------------------- */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);

    tx_buf[0] = FC_POLL;
    for (int i = 1; i < 16; i++) tx_buf[i] = 0x00;

    dw1000_SetDataToTransmit(&dw1000, tx_buf, 16, 1);
    dw1000_StartTransmit(&dw1000, 16, 1);

    uint8_t sent = 0;
    uint8_t sys_status[5];
    if (DW1000_WaitForIrqStatus(100, sys_status)) {
      if ((sys_status[0] >> 7) & 1) { sent = 1; }
    }

    if (!sent) {
      UART_Print("E:poll_tx_timeout\r\n");
      DW1000_Force_TRXOFF(&dw1000);
      TAG_RetryBackoff();
      continue;
    }

    t_poll_tx = dw1000_GetTxTimestamp(&dw1000);  /* T1 */

    /* ---- 2. Wait for RESPONSE (T4) ------------------------------------- */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    uint32_t rx_start = HAL_GetTick();
    uint8_t rx_success = 0;

    while (HAL_GetTick() - rx_start < 300) {
      if (!DW1000_WaitForIrqStatus(300 - (HAL_GetTick() - rx_start), sys_status)) {
        break;
      }

      uint8_t rxdfr  = (sys_status[1] >> 5) & 1;
      uint8_t rxfcg  = (sys_status[1] >> 6) & 1;
      uint8_t rxfce  = (sys_status[1] >> 7) & 1;
      uint8_t rxprej = (sys_status[4] >> 1) & 1;

      if (rxprej) {
        /* Count but don't print — we're in the middle of a cycle */
        DW1000_Force_TRXOFF(&dw1000);
        DW1000_ClearAllStatus_Manual(&dw1000);
        dw1000_StartReceive(&dw1000, 1);
        continue;
      }
      if (rxfce) {
        DW1000_Force_TRXOFF(&dw1000);
        DW1000_ClearAllStatus_Manual(&dw1000);
        dw1000_StartReceive(&dw1000, 1);
        continue;
      }
      if (rxdfr && rxfcg) {
        /* Capture T4 immediately — before GetDataReceived */
        t_resp_rx = dw1000_GetRxTimestamp(&dw1000);
        rx_success = 1;
        break;
      }
    }

    if (!rx_success) {
      UART_Print("E:resp_rx_timeout\r\n");
      DW1000_Force_TRXOFF(&dw1000);
      TAG_RetryBackoff();
      continue;
    }

    uint16_t len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len > sizeof(rx_buf)) len = sizeof(rx_buf);
    dw1000_GetDataReceived(&dw1000, rx_buf, len);

    if (rx_buf[0] != FC_RESPONSE) {
      UART_Print("E:wrong_fc\r\n");
      TAG_RetryBackoff();
      continue;
    }

    /* ---- 3. Send FINAL with T1, T4, T5 — NO UART between T4 and TX ---- *
     * UART_Print here would burn ~4ms of FINAL_TX_DELAY_US budget.        *
     * Keep this block completely silent.                                   */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);

    dw1000_timestamp_t t_final_planned =
        dw1000_PlanDelayedTransmit(&dw1000, FINAL_TX_DELAY_US);

    tx_buf[0] = FC_FINAL;
    for (int b = 0; b < 5; b++)
      tx_buf[1 + b]  = (uint8_t)(t_poll_tx       >> (8 * b));  /* T1 */
    for (int b = 0; b < 5; b++)
      tx_buf[6 + b]  = (uint8_t)(t_resp_rx        >> (8 * b)); /* T4 */
    for (int b = 0; b < 5; b++)
      tx_buf[11 + b] = (uint8_t)(t_final_planned  >> (8 * b)); /* T5 */

    dw1000_SetDataToTransmit(&dw1000, tx_buf, 16, 1);
    dw1000_StartDelayedTransmit(&dw1000, 16, 1);

    sent = 0;
    uint8_t hpdwarn = 0;
    if (DW1000_WaitForIrqStatus(100, sys_status)) {
      if ((sys_status[3] >> 3) & 1) hpdwarn = 1;
      if ((sys_status[0] >> 7) & 1) { sent = 1; }
    }

    /* UART is safe again after the frame has fired */
    if (hpdwarn) {
      UART_Print("E:hpdwarn\r\n");
    }

    if (!sent) {
      UART_Print("E:final_tx_timeout\r\n");
      DW1000_Force_TRXOFF(&dw1000);
      TAG_RetryBackoff();
      continue;
    }

    dw1000_timestamp_t round1 = (t_resp_rx - t_poll_tx) & DW1000_TS_MASK;
    snprintf(msg, sizeof(msg), "T:ok,r1=%lu\r\n", (uint32_t)(round1 & 0xFFFFFFFF));
    UART_Print(msg);
  }
}

/* --- HARDWARE FUNCTIONS (unchanged from original) --- */

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
}

static void DW1000_ClearAllStatus_Manual(dw1000_HandleTypeDef *dw1000) {
    uint8_t clear_status[5] = {0xFC, 0xFF, 0xFF, 0xFF, 0xFF};
    dw1000_WriteData(dw1000, 0x0F, clear_status, 5);
    dw1000_irq_pending = 0;
}

static void DW1000_DisableFrameFiltering(dw1000_HandleTypeDef *dw1000) {
    uint8_t sys_cfg[4];
    dw1000_ReadData(dw1000, 0x04, sys_cfg, 4);
    sys_cfg[0] &= ~0x01;
    dw1000_WriteData(dw1000, 0x04, sys_cfg, 4);
}

static void UART_Print(const char *str) {
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0u || uart_tx_busy) {
        return;
    }
    if (len > sizeof(uart_tx_dma_buf)) {
        len = sizeof(uart_tx_dma_buf);
    }
    memcpy(uart_tx_dma_buf, str, len);
    uart_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart2, uart_tx_dma_buf, len) != HAL_OK) {
        uart_tx_busy = 0;
    }
}

static void DW1000_EnableEventIrq(dw1000_HandleTypeDef *dw1000) {
    uint8_t sys_cfg[4];
    dw1000_ReadData(dw1000, DW1000_SYS_CFG, sys_cfg, sizeof(sys_cfg));
    sys_cfg[1] |= 0x02u;  /* HIRQ_POL: active-high IRQ for rising-edge EXTI. */
    dw1000_WriteData(dw1000, DW1000_SYS_CFG, sys_cfg, sizeof(sys_cfg));

    /* SYS_MASK bits: TXFRS, RXPHE, RXFCG, RXFCE, RXRFSL, LDEERR,
     * RXSFDTO, HPDWARN. RXPREJ is above bit 31 and is still checked after
     * RX-related IRQs when SYS_STATUS is read. */
    uint8_t sys_mask[4] = {0};
    sys_mask[0] = (uint8_t)(1u << 7);                         /* TXFRS */
    sys_mask[1] = (uint8_t)((1u << 4) | (1u << 6) | (1u << 7));/* RXPHE/RXFCG/RXFCE */
    sys_mask[2] = (uint8_t)((1u << 0) | (1u << 2));            /* RXRFSL/LDEERR */
    sys_mask[3] = (uint8_t)((1u << 2) | (1u << 3));            /* RXSFDTO/HPDWARN */
    dw1000_WriteData(dw1000, DW1000_SYS_MASK, sys_mask, sizeof(sys_mask));
}

static uint8_t DW1000_WaitForIrqStatus(uint32_t timeout_ms, uint8_t *sys_status) {
    uint32_t start = HAL_GetTick();
    if (timeout_ms == 0u) {
        timeout_ms = 1u;
    }

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (dw1000_irq_pending ||
            HAL_GPIO_ReadPin(DW1000_IRQ_PORT, DW1000_IRQ_PIN) == GPIO_PIN_SET) {
            dw1000_irq_pending = 0;
            dw1000_ReadData(&dw1000, DW1000_SYS_STATUS, sys_status, 5);
            if ((sys_status[0] & 0x80u) || (sys_status[1] & 0xD0u) ||
                (sys_status[2] & 0x05u) || (sys_status[3] & 0x0Cu) ||
                (sys_status[4] & 0x02u)) {
                return 1;
            }
        }
        __WFI();
    }

    return 0;
}

static void TAG_RetryBackoff(void) {
    if (TAG_ERROR_BACKOFF_MS > 0u) {
        HAL_Delay(TAG_ERROR_BACKOFF_MS);
    }
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
           ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] <<  8) |
            (dw1000_timestamp_t)buffer[0];
}

static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw1000) {
    uint8_t buffer[5];
    dw1000_ReadData(dw1000, DW1000_RX_TIME, buffer, 5);
    return ((dw1000_timestamp_t)buffer[4] << 32) | ((dw1000_timestamp_t)buffer[3] << 24) |
           ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] <<  8) |
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

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart2_tx.Instance = DMA1_Channel1;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
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
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == DW1000_IRQ_PIN) {
        dw1000_irq_pending = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        uart_tx_busy = 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        uart_tx_busy = 0;
    }
}

void EXTI4_15_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(DW1000_IRQ_PIN);
}

void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void Error_Handler(void) { __disable_irq(); while (1) { } }
