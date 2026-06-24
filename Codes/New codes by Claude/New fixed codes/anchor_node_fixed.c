/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_ANCHOR.c
  * @brief          : DW1000 DS-TWR ANCHOR — fixed for sawtooth pattern
  *
  * Changes from uploaded anchor_node.c:
  *
  *   [B] t_poll_rx (T2) now captured IMMEDIATELY after rxdfr&&rxfcg, before
  *       any SPI read of RX_FINFO/RX_BUFFER. RX_TIME is hardware-latched
  *       and independent of those reads, so it was safe before — but this
  *       ordering is defensively correct and makes the intent explicit.
  *
  *   [DS-TWR] Formula upgraded from single-sided to double-sided TWR:
  *       ToF = (round1*round2 - reply1*reply2) / (round1+round2+reply1+reply2)
  *       where:
  *         round1 = T4-T1  (TAG, from FINAL payload bytes [1..5])
  *         reply2 = T5-T4  (TAG, from FINAL payload bytes [6..10], previous cycle)
  *         reply1 = T3-T2  (ANCHOR, computed here)
  *         round2 = T6-T3  (ANCHOR, t_final_rx - t_resp_tx)
  *       t_final_rx (T6) is captured IMMEDIATELY after rxdfr&&rxfcg on FINAL,
  *       before any other SPI access.
  *       First cycle is skipped (reply2==0, not yet available from TAG).
  *       Uses __int128 for numerator products to avoid uint64 overflow at 110kbps.
  *
  *   [G] Dead variables t_reply_prev / have_prev_reply removed.
  *   [CALIB] CALIBRATION_OFFSET_M reset to 0.0 — re-measure at known distance
  *       after DS-TWR is running, as the old 168.43m offset was compensating
  *       for single-sided TWR clock-drift error, not physical antenna delay.
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

static void DW1000_ClearAllStatus_Manual(dw1000_HandleTypeDef *dw1000);
static void DW1000_Force_TRXOFF(dw1000_HandleTypeDef *dw1000);
static void DW1000_DisableFrameFiltering(dw1000_HandleTypeDef *dw1000);
static void DW1000_LoadLDE_Microcode(dw1000_HandleTypeDef *dw1000);

#define FC_POLL       0x20
#define FC_RESPONSE   0x21
#define FC_FINAL      0x22

/* Re-calibrate this at a known physical distance after DS-TWR is running.
 * The old 168.43 m compensated for single-sided TWR clock-drift bias
 * which DS-TWR eliminates — so the old value will massively over-subtract. */
#define CALIBRATION_OFFSET_M  0.0

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

  /* DS-TWR timestamps — ANCHOR side.
   *   T2 = t_poll_rx   ANCHOR receives POLL
   *   T3 = t_resp_tx   ANCHOR transmits RESPONSE
   *   T6 = t_final_rx  ANCHOR receives FINAL
   * TAG sends T4-T1 (round1) and T5-T4 (reply2, previous cycle) in FINAL. */
  dw1000_timestamp_t t_poll_rx  = 0;
  dw1000_timestamp_t t_resp_tx  = 0;
  dw1000_timestamp_t t_final_rx = 0;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  dw1000.spi     = &hspi1;
  dw1000.ss_port = DW1000_SS_PORT;
  dw1000.ss_pin  = DW1000_SS_PIN;

  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);
  DW1000_HardReset(&dw1000);

  dw1000_Init(&dw1000, 5, DW1000_PRF_16MHZ, DW1000_TX_FCTRL_TXBR_110K);

  dw1000_pan_addr_t pan_addr;
  pan_addr.pan_id     = 0xDECA;
  pan_addr.short_addr = 0x0001;
  dw1000_SetPanAddress(&dw1000, &pan_addr);

  DW1000_LoadLDE_Microcode(&dw1000);
  DW1000_Force_TRXOFF(&dw1000);
  DW1000_DisableFrameFiltering(&dw1000);
  DW1000_ClearAllStatus_Manual(&dw1000);

  UART_Print("\r\n========================================\r\n");
  UART_Print("[ANCHOR] DS-TWR initialized. Awaiting POLL...\r\n");
  UART_Print("========================================\r\n");

  uint32_t last_heartbeat = HAL_GetTick();

  while (1)
  {
    /* ---- 1. Wait for POLL (capture T2) ------------------- */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    uint8_t rx_success    = 0;
    uint8_t preamble_notified = 0;

    while (!rx_success)
    {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);

      uint8_t rxprd   = (sys_status[1] >> 0) & 1;
      uint8_t rxdfr   = (sys_status[1] >> 5) & 1;
      uint8_t rxfcg   = (sys_status[1] >> 6) & 1;
      uint8_t rxfce   = (sys_status[1] >> 7) & 1;
      uint8_t rxphe   = (sys_status[1] >> 4) & 1;
      uint8_t rxrfsl  = (sys_status[2] >> 0) & 1;
      uint8_t ldeerr  = (sys_status[2] >> 2) & 1;
      uint8_t rxsfdto = (sys_status[3] >> 2) & 1;
      uint8_t rxprej  = (sys_status[4] >> 1) & 1;

      if (HAL_GetTick() - last_heartbeat > 2000) {
        UART_Print("[ANCHOR] Listening... (no signal)\r\n");
        last_heartbeat = HAL_GetTick();
      }

      if (rxprd && !preamble_notified) {
        UART_Print("[ANCHOR] -> Preamble detected. Waiting for SFD...\r\n");
        preamble_notified = 1;
      }

      if (rxprej) {
        UART_Print("[ANCHOR] -> ERROR: Preamble Rejected (RXPREJ).\r\n");
        DW1000_Force_TRXOFF(&dw1000);
        DW1000_ClearAllStatus_Manual(&dw1000);
        dw1000_StartReceive(&dw1000, 1);
        preamble_notified = 0;
        continue;
      }

      if (rxfce || rxphe || rxrfsl || rxsfdto || ldeerr) {
        if (ldeerr)  UART_Print("[ANCHOR] -> ERROR: LDE crashed.\r\n");
        if (rxsfdto) UART_Print("[ANCHOR] -> ERROR: SFD Timeout.\r\n");
        if (rxphe)   UART_Print("[ANCHOR] -> ERROR: PHY Header error.\r\n");
        DW1000_Force_TRXOFF(&dw1000);
        DW1000_ClearAllStatus_Manual(&dw1000);
        dw1000_StartReceive(&dw1000, 1);
        preamble_notified = 0;
        continue;
      }

      if (rxdfr && rxfcg) {
        /* [FIX B] Capture T2 immediately — before any other SPI access.
         * RX_TIME is latched by hardware at LDEDONE and independent of
         * RX_BUFFER reads, but capturing it first is the correct intent
         * and eliminates all ordering ambiguity. */
        t_poll_rx = dw1000_GetRxTimestamp(&dw1000);
        rx_success = 1;
      }
    }

    uint16_t len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len > sizeof(rx_buf)) len = sizeof(rx_buf);
    dw1000_GetDataReceived(&dw1000, rx_buf, len);

    if (rx_buf[0] != FC_POLL) continue;

    UART_Print("[ANCHOR] >> RX POLL (T2 captured). Sending RESPONSE...\r\n");

    /* ---- 2. Send RESPONSE (capture T3) ------------------- */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);

    tx_buf[0] = FC_RESPONSE;
    for (int i = 1; i < 16; i++) tx_buf[i] = 0x00;

    dw1000_SetDataToTransmit(&dw1000, tx_buf, 16, 1);
    dw1000_StartTransmit(&dw1000, 16, 1);

    uint32_t tx_start = HAL_GetTick();
    uint8_t tx_sent = 0;
    while (HAL_GetTick() - tx_start < 100) {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);
      if ((sys_status[0] >> 7) & 1) { tx_sent = 1; break; }
    }

    if (!tx_sent) {
      UART_Print("[ANCHOR] TX RESPONSE timeout.\r\n");
      continue;
    }

    t_resp_tx = dw1000_GetTxTimestamp(&dw1000);  /* T3 */
    dw1000_timestamp_t reply1 = t_resp_tx - t_poll_rx;  /* T3-T2 */

    UART_Print("[ANCHOR] << RESPONSE sent (T3 captured). Waiting for FINAL...\r\n");

    /* ---- 3. Wait for FINAL (capture T6) ------------------ */
    DW1000_Force_TRXOFF(&dw1000);
    DW1000_ClearAllStatus_Manual(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    uint32_t rx_final_start = HAL_GetTick();
    rx_success = 0;

    while (HAL_GetTick() - rx_final_start < 250) {
      uint8_t sys_status[5];
      dw1000_ReadData(&dw1000, 0x0F, sys_status, 5);
      if (((sys_status[1] >> 5) & 1) && ((sys_status[1] >> 6) & 1)) {
        /* Capture T6 immediately — before reading frame data */
        t_final_rx = dw1000_GetRxTimestamp(&dw1000);
        rx_success = 1;
        break;
      }
    }

    if (!rx_success) {
      UART_Print("[ANCHOR] RX FINAL timeout! Cycle broken.\r\n");
      DW1000_Force_TRXOFF(&dw1000);
      continue;
    }

    len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len > sizeof(rx_buf)) len = sizeof(rx_buf);
    dw1000_GetDataReceived(&dw1000, rx_buf, len);

    if (rx_buf[0] != FC_FINAL) continue;

    UART_Print("[ANCHOR] >> RX FINAL (T6 captured).\r\n");

    /* ---- 4. DS-TWR distance computation ------------------ */
    /*
     * FINAL payload layout (TAG sends this):
     *   bytes [1..5]  = round1 = T4-T1  (current cycle, TAG clock)
     *   bytes [6..10] = reply2 = T5-T4  (PREVIOUS cycle, TAG clock)
     *
     * TAG cannot know T5 before transmitting FINAL, so reply2 is
     * always from the previous cycle. First cycle: reply2==0, skip.
     */
    dw1000_timestamp_t round1 = 0;
    dw1000_timestamp_t reply2 = 0;
    for (int b = 0; b < 5; b++)
      round1 |= ((dw1000_timestamp_t)rx_buf[1 + b]) << (8 * b);
    for (int b = 0; b < 5; b++)
      reply2 |= ((dw1000_timestamp_t)rx_buf[6 + b]) << (8 * b);

    dw1000_timestamp_t round2 = t_final_rx - t_resp_tx;  /* T6-T3 */

    sprintf(msg, "[ANCHOR] round1=%lu  reply1=%lu  round2=%lu  reply2=%lu\r\n",
            (uint32_t)(round1 & 0xFFFFFFFF), (uint32_t)(reply1 & 0xFFFFFFFF),
            (uint32_t)(round2 & 0xFFFFFFFF), (uint32_t)(reply2 & 0xFFFFFFFF));
    UART_Print(msg);

    if (reply2 == 0) {
      UART_Print("[ANCHOR] First cycle — reply2 not yet available. Skipping.\r\n");
      continue;
    }

    /*
     * DS-TWR formula (Decawave AN005, eq. 8):
     *   ToF = (round1*round2 - reply1*reply2) / (round1+round2+reply1+reply2)
     *
     * Use __int128 to avoid overflow: at 110 kbps with ~200ms reply delays,
     * round1 ~ 1.28e10 ticks; round1*round2 ~ 1.6e20 which exceeds uint64.
     * __int128 is available on ARM GCC and adds no library dependency.
     */
    typedef __int128 int128_t;
    int128_t R1 = (int128_t)(int64_t)round1;
    int128_t R2 = (int128_t)(int64_t)round2;
    int128_t P1 = (int128_t)(int64_t)reply1;
    int128_t P2 = (int128_t)(int64_t)reply2;

    int128_t numerator   = R1 * R2 - P1 * P2;
    int128_t denominator = R1 + R2 + P1 + P2;

    if (denominator == 0) {
      UART_Print("[ANCHOR] ERROR: denominator zero.\r\n");
      continue;
    }

    int64_t tof_raw = (int64_t)(numerator / denominator);

    if (tof_raw < 0) {
      UART_Print("[ANCHOR] Negative ToF — clock wrap or bad cycle.\r\n");
      continue;
    }

    double distance_m = DW1000_Time_TimestampToMeters((dw1000_timestamp_t)tof_raw);
    distance_m -= CALIBRATION_OFFSET_M;

    int dist_int  = (int)distance_m;
    int dist_frac = (int)((distance_m - (double)dist_int) * 1000);
    if (dist_frac < 0) dist_frac = -dist_frac;

    if (distance_m > 0.0 && distance_m < 1000.0) {
      sprintf(msg, "[ANCHOR] *** DS-TWR Distance: %d.%03d m ***\r\n", dist_int, dist_frac);
    } else {
      sprintf(msg, "[ANCHOR] Distance OOB: %d.%03d m (needs calibration?)\r\n", dist_int, dist_frac);
    }
    UART_Print(msg);
  }
}

/* --- HARDWARE FIX FUNCTIONS (unchanged) --- */

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
