/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_DW1000Ranging_ANCHOR.c
  * @brief          : DW1000 Simple Two-Way Ranging  -  ANCHOR role
  *                   NUCLEO-G070RB
  ******************************************************************************
  * Pin map (user wiring):
  *   DWM1000 SPICLK  (pin 20) -> PA5  (D13, SPI1_SCK)
  *   DWM1000 SPIMISO (pin 19) -> PA6  (D12, SPI1_MISO)
  *   DWM1000 SPIMOSI (pin 18) -> PA7  (D11, SPI1_MOSI)
  *   DWM1000 SPICSn  (pin 17) -> PB0  (D10, manual GPIO CS)
  *   DWM1000 IRQ     (pin 22) -> PA9  (D8,  input, polled)
  *   DWM1000 RSTn    (pin  3) -> PA8  (D7,  output, active-low reset)
  ******************************************************************************
  *
  * Key sequencing notes (cross-referenced to DWM1000 datasheet):
  *
  * 1. SPI speed MUST be <= 3 MHz until CPLOCK asserts (datasheet Table 2).
  *    We use /32 (2 MHz) for all init writes, then /8 (8 MHz) after CPLOCK.
  *
  * 2. After hard reset, RSTn is driven LOW internally for ~3 ms (TDIG_ON,
  *    datasheet Table 1).  We wait 10 ms after deasserting to be safe.
  *
  * 3. dw1000_Init() does: PMSC soft-reset, LDE microcode load, full
  *    AGC/DRX/RF/FS_CTRL tuning for Ch5 / PRF16 / 110 kbps per the
  *    DW1000 User Manual "Default configurations that must be changed".
  *
  * 4. CPLOCK (SYS_STATUS bit 1) must be set before any RF activity.
  *    We poll it with a 200 ms timeout and print the result.
  *
  * 5. Before every RX enable we call dw1000_IdleMode() to ensure the
  *    transceiver state machine is in IDLE/TRXOFF before transitioning.
  *
  * 6. We poll RXFCG (bit 14, "FCS Good") not RXDFR (bit 13, "Data
  *    Frame Ready") — RXFCG guarantees valid CRC, RXDFR does not.
  *
  * 7. All polling loops use HAL_GetTick()-based timeouts, not raw loop
  *    counts, for reproducible behaviour regardless of compiler options.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "dw1000.h"
#include "dw1000_stm32.h"
#include "dw1000_time.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init_Slow(void);   /* /32 -> 2 MHz, used before CPLOCK */
static void MX_SPI1_Init_Fast(void);   /* /8  -> 8 MHz, used after  CPLOCK */
static void MX_USART2_UART_Init(void);

static void     DW1000_HardReset(void);
static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw);
static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw);
static void     UART_Print(const char *str);

/* ---- Application frame codes (must match TAG) ----
 *   FC_POLL     0x20  Tag -> Anchor
 *   FC_RESPONSE 0x21  Anchor -> Tag
 *   FC_FINAL    0x22  Tag -> Anchor  (carries prev-cycle t_round, LE 40-bit)
 */
#define FC_POLL       0x20u
#define FC_RESPONSE   0x21u
#define FC_FINAL      0x22u

/* ---- GPIO assignments (per user wiring) ---- */
#define DW1000_SS_PIN   GPIO_PIN_0    /* PB0 (D10) SPI chip-select */
#define DW1000_SS_PORT  GPIOB
#define DW1000_RST_PIN  GPIO_PIN_8    /* PA8 (D7)  RSTn            */
#define DW1000_RST_PORT GPIOA
#define DW1000_IRQ_PIN  GPIO_PIN_9    /* PA9 (D8)  IRQ, polled     */
#define DW1000_IRQ_PORT GPIOA

/* ---- Timeout values (milliseconds) ---- */
#define TIMEOUT_CPLOCK_MS   200u  /* generous; crystal startup ~3 ms */
#define TIMEOUT_TX_MS        50u  /* TXFRS should set within 1-2 ms  */
#define TIMEOUT_RX_POLL_MS  500u  /* TAG sends POLL every ~200 ms     */
#define TIMEOUT_RX_FINAL_MS 200u  /* FINAL arrives very soon after RESPONSE TX */

static dw1000_HandleTypeDef dw1000;

/* ========================================================================== */
int main(void)
{
  uint8_t  rx_buf[16];
  uint8_t  tx_buf[16];
  char     msg[200];
  uint8_t  sys_status[DW1000_SYS_STATUS_LEN];

  dw1000_timestamp_t t_poll_rx   = 0;
  dw1000_timestamp_t t_resp_tx   = 0;
  dw1000_timestamp_t t_reply_prev = 0;
  dw1000_timestamp_t t_round     = 0;
  uint8_t  have_prev_reply = 0;
  uint8_t  frame_received;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* ---- Step A: Hard reset DW1000 ------------------------------------------
   * RSTn is driven low by the MCU; the DW1000 also drives it low internally
   * for ~3 ms (TDIG_ON, datasheet Table 1).  We hold it low 5 ms then
   * release and wait 10 ms for crystal startup before any SPI access.
   * Note: datasheet says "RSTn should NEVER be driven HIGH by an external
   * source" — we only drive it LOW then release to Hi-Z / open-drain.      */
  DW1000_HardReset();

  /* ---- Step B: Start SPI at SLOW speed (2 MHz) ----------------------------
   * DWM1000 datasheet Table 2: max SPI = 3 MHz before CLKPLL locked.       */
  MX_SPI1_Init_Slow();

  dw1000.spi    = &hspi1;
  dw1000.ss_port = DW1000_SS_PORT;
  dw1000.ss_pin  = DW1000_SS_PIN;

  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET); /* CS idle high */

  /* ---- Step C: Verify SPI link with device ID read ------------------------ */
  dw1000_dev_id_t dev_id = dw1000_GetDevID(&dw1000);
  sprintf(msg, "\r\n[ANCHOR] DEV_ID model=0x%02X ver=%u rev=%u ridtag=0x%04X %s\r\n",
          dev_id.model, dev_id.ver, dev_id.rev, dev_id.ridtag,
          (dev_id.ridtag == 0xDECA) ? "(OK)" : "(WRONG - SPI problem?)");
  UART_Print(msg);

  /* ---- Step D: Full RF init / tuning sequence (2 MHz SPI) -----------------
   * Writes: PMSC soft-reset, LDE microcode load, AGC_TUNE1/2/3,
   * DRX_TUNE0b/1a/1b/2/4H, LDE_CFG1/2, RF_RXCTRLH, RF_TXCTRL,
   * TC_PGDELAY, FS_PLLTUNE/PLLCFG/XTALT, TX_POWER, TX_FCTRL rate bits,
   * SYS_CFG.  All at 2 MHz to stay within pre-PLL SPI limit.              */
  dw1000_pan_addr_t pan_addr = { .pan_id = 0xDECA, .short_addr = 0x0001 };
  dw1000_SetPanAddress(&dw1000, &pan_addr);

  dw1000_channel_t chan = {
    .tx_chan = 5, .rx_chan = 5,
    .prf = DW1000_PRF_16MHZ,
    .tx_preamble = 0x4,   /* 1024-symbol preamble code 4 */
    .rx_preamble = 0x4
  };
  dw1000_SetChannel(&dw1000, &chan);

  uint8_t init_ok = dw1000_Init(&dw1000, 5, DW1000_PRF_16MHZ,
                                 DW1000_TX_FCTRL_TXBR_110K);
  sprintf(msg, "[ANCHOR] dw1000_Init() %s\r\n",
          init_ok ? "OK" : "FAILED (sub-reg readback mismatch)");
  UART_Print(msg);

  /* Tuning register readback for diagnostics */
  uint16_t agc1 = dw1000_ReadSubReg16(&dw1000, DW1000_AGC_CTRL, DW1000_AGC_TUNE1_SUB);
  uint8_t  lde1 = dw1000_ReadSubReg8 (&dw1000, DW1000_LDE_CTRL, DW1000_LDE_CFG1_SUB);
  uint32_t drx2 = dw1000_ReadSubReg32(&dw1000, DW1000_DRX_CONF, DW1000_DRX_TUNE2_SUB);
  sprintf(msg, "[ANCHOR] AGC_TUNE1=0x%04X(exp 0x8870) LDE_CFG1=0x%02X(exp 0x6D)"
               " DRX_TUNE2=0x%08lX(exp 0x311A002D)\r\n",
          agc1, lde1, (unsigned long)drx2);
  UART_Print(msg);

  /* ---- Step E: Wait for RF PLL to lock (CPLOCK) ---------------------------
   * THIS IS MANDATORY before any TX or RX.  The RF section is not
   * operational until CPLOCK asserts.  Without it: TXFRS never fires,
   * RXFCG never fires — exactly the symptoms you observed.                  */
  UART_Print("[ANCHOR] Waiting for CPLOCK...\r\n");
  uint8_t cplock_ok = dw1000_WaitForCPLOCK(&dw1000, TIMEOUT_CPLOCK_MS);
  dw1000_GetSystemStatus(&dw1000, sys_status);
  sprintf(msg, "[ANCHOR] CPLOCK %s. SYS_STATUS[0..4]=%02X %02X %02X %02X %02X\r\n",
          cplock_ok ? "LOCKED" : "TIMEOUT (RF dead - check power/wiring)",
          sys_status[0], sys_status[1], sys_status[2], sys_status[3], sys_status[4]);
  UART_Print(msg);

  if (!cplock_ok) {
    UART_Print("[ANCHOR] FATAL: CPLOCK never asserted. Halting.\r\n");
    while (1) { HAL_Delay(1000); UART_Print("[ANCHOR] Still no CPLOCK.\r\n"); }
  }

  /* ---- Step F: Switch SPI to fast speed (8 MHz) after PLL lock ----------- */
  MX_SPI1_Init_Fast();
  UART_Print("[ANCHOR] SPI switched to 8 MHz (PLL locked).\r\n");

  /* Channel/PAN config confirmation readback */
  dw1000_pan_addr_t pan_rb  = dw1000_GetPanAddress(&dw1000);
  dw1000_channel_t  chan_rb = dw1000_GetChannel(&dw1000);
  sprintf(msg, "[ANCHOR] PAN=0x%04X ADDR=0x%04X CH tx=%u rx=%u PRF=%u\r\n",
          pan_rb.pan_id, pan_rb.short_addr,
          chan_rb.tx_chan, chan_rb.rx_chan, chan_rb.prf);
  UART_Print(msg);

  dw1000_ClearAllStatus(&dw1000);
  UART_Print("[ANCHOR] Ready. Listening for POLL frames...\r\n");

  /* ==========================================================================
   * Main ranging loop
   * Protocol (Single-Sided TWR, piggyback variant):
   *   Cycle N:  Tag->Anchor POLL  /  Anchor->Tag RESPONSE  /  Tag->Anchor FINAL
   *   FINAL carries t_round from cycle N-1 (LE 40-bit, bytes 1..5).
   *   Anchor pairs each FINAL's t_round with the t_reply from cycle N-1.
   * ========================================================================== */
  while (1)
  {
    /* ---- 1. Enable receiver for POLL ------------------------------------ */
    dw1000_IdleMode(&dw1000);           /* ensure TRXOFF first */
    HAL_Delay(1);
    dw1000_ClearReceiveStatus(&dw1000);
    dw1000_ClearAllStatus(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    frame_received = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TIMEOUT_RX_POLL_MS) {
      if (dw1000_ReceiveDataFrameReady(&dw1000)) { /* polls RXFCG */
        frame_received = 1;
        break;
      }
    }

    if (!frame_received) {
      /* Print status dump every timeout to show what the receiver sees */
      dw1000_GetSystemStatus(&dw1000, sys_status);
      uint32_t state = dw1000_GetSystemState(&dw1000);
      sprintf(msg, "[ANCHOR] RX timeout. STATUS=%02X%02X%02X%02X%02X STATE=0x%06lX "
                   "CPLOCK=%u RXPRD=%u RXSFDD=%u RXPHE=%u RXFCE=%u RXRFSL=%u "
                   "LDEERR=%u RXSFDTO=%u RXFCG=%u\r\n",
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
      continue;
    }

    uint16_t len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len == 0 || len > sizeof(rx_buf)) {
      sprintf(msg, "[ANCHOR] Bad RX len=%u\r\n", len);
      UART_Print(msg);
      continue;
    }
    dw1000_GetDataReceived(&dw1000, rx_buf, len);
    dw1000_ClearReceiveStatus(&dw1000);

    sprintf(msg, "[ANCHOR] RX ok len=%u fc=0x%02X\r\n", len, rx_buf[0]);
    UART_Print(msg);

    if (rx_buf[0] != FC_POLL) {
      UART_Print("[ANCHOR] Not POLL, ignoring.\r\n");
      continue;
    }

    t_poll_rx = dw1000_GetRxTimestamp(&dw1000);

    /* ---- 2. Send RESPONSE ----------------------------------------------- */
    dw1000_IdleMode(&dw1000);
    HAL_Delay(1);
    tx_buf[0] = FC_RESPONSE;
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 1, 1);
    dw1000_ClearTransmitStatus(&dw1000);
    dw1000_StartTransmit(&dw1000, 1, 1);

    uint8_t sent = 0;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TIMEOUT_TX_MS) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      if (dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXFRS)) {
        sent = 1;
        break;
      }
    }
    if (!sent) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      uint32_t state = dw1000_GetSystemState(&dw1000);
      sprintf(msg, "[ANCHOR] RESPONSE TX timeout. STATUS=%02X%02X%02X%02X%02X "
                   "STATE=0x%06lX TXBERR=%u\r\n",
              sys_status[4],sys_status[3],sys_status[2],sys_status[1],sys_status[0],
              (unsigned long)state,
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXBERR));
      UART_Print(msg);
      continue;
    }

    t_resp_tx = dw1000_GetTxTimestamp(&dw1000);
    dw1000_ClearTransmitStatus(&dw1000);
    dw1000_timestamp_t t_reply_this = t_resp_tx - t_poll_rx;

    sprintf(msg, "[ANCHOR] RESPONSE sent. t_reply_this=%llu\r\n",
            (unsigned long long)t_reply_this);
    UART_Print(msg);

    /* ---- 3. Receive FINAL ----------------------------------------------- */
    dw1000_IdleMode(&dw1000);
    HAL_Delay(1);
    dw1000_ClearReceiveStatus(&dw1000);
    dw1000_ClearAllStatus(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    frame_received = 0;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TIMEOUT_RX_FINAL_MS) {
      if (dw1000_ReceiveDataFrameReady(&dw1000)) {
        frame_received = 1;
        break;
      }
    }
    if (!frame_received) {
      dw1000_GetSystemStatus(&dw1000, sys_status);
      sprintf(msg, "[ANCHOR] FINAL RX timeout. STATUS=%02X%02X%02X%02X%02X "
                   "RXPHE=%u RXFCE=%u RXRFSL=%u LDEERR=%u RXSFDTO=%u\r\n",
              sys_status[4],sys_status[3],sys_status[2],sys_status[1],sys_status[0],
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXPHE),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXFCE),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXRFSL),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_LDEERR),
              dw1000_IsSet(sys_status, DW1000_SYS_STATUS_RXSFDTO));
      UART_Print(msg);
      continue;
    }

    len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len < 6 || len > sizeof(rx_buf)) {
      sprintf(msg, "[ANCHOR] FINAL bad len=%u\r\n", len);
      UART_Print(msg);
      dw1000_ClearReceiveStatus(&dw1000);
      continue;
    }
    dw1000_GetDataReceived(&dw1000, rx_buf, len);
    dw1000_ClearReceiveStatus(&dw1000);

    sprintf(msg, "[ANCHOR] RX ok len=%u fc=0x%02X\r\n", len, rx_buf[0]);
    UART_Print(msg);

    if (rx_buf[0] != FC_FINAL) {
      UART_Print("[ANCHOR] Not FINAL, ignoring.\r\n");
      have_prev_reply = 1; t_reply_prev = t_reply_this;
      continue;
    }

    /* Unpack t_round from FINAL payload (LE 40-bit, bytes 1..5) */
    t_round = 0;
    for (int b = 0; b < 5; b++)
      t_round |= ((dw1000_timestamp_t)rx_buf[1 + b]) << (8 * b);

    /* ---- 4. Compute distance (prev cycle) --------------------------------
     * SS-TWR:  tof = (t_round_prev - t_reply_prev) / 2                     */
    if (have_prev_reply && t_round != 0) {
      dw1000_timestamp_t tof_raw  = (t_round - t_reply_prev) / 2;
      double tof_us   = DW1000_Time_TimestampToMicroseconds(tof_raw);
      double distance = DW1000_Time_TimestampToMeters(tof_raw);
      sprintf(msg, "[ANCHOR] t_round=%llu t_reply_prev=%llu "
                   "ToF=%.4f us  Distance=%.3f m\r\n",
              (unsigned long long)t_round,
              (unsigned long long)t_reply_prev,
              tof_us, distance);
      UART_Print(msg);
    } else {
      UART_Print("[ANCHOR] FINAL ok, first cycle t_round=0 - skipping.\r\n");
    }

    have_prev_reply = 1;
    t_reply_prev = t_reply_this;
  }
}

/* ============================================================================
 * Helper implementations
 * ========================================================================== */
static void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void DW1000_HardReset(void)
{
  /* Drive RSTn LOW (active low reset).  Per datasheet the minimum
   * assertion is 10 ns; we use 5 ms to be conservative.
   * IMPORTANT: must never drive RSTn HIGH — release to Hi-Z instead.
   * Since our GPIO is push-pull and must stay low-enough during reset we
   * drive low then switch the pin back to Hi-Z (input mode) after. */
  GPIO_InitTypeDef g = {0};

  /* Drive low */
  g.Pin   = DW1000_RST_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW1000_RST_PORT, &g);
  HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);

  /* Release to input (Hi-Z) — do NOT drive high */
  g.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_RST_PORT, &g);
  HAL_Delay(10);   /* crystal startup: TDIG_ON ~3 ms (Table 1); 10 ms safe */
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

/* ============================================================================
 * HAL peripheral init
 * ========================================================================== */
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

/* SPI1 at /32 -> 64MHz/32 = 2 MHz  (below 3 MHz limit before CPLOCK) */
static void MX_SPI1_Init_Slow(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;   /* CPOL=0 */
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;    /* CPHA=0 -> SPI mode 0 */
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

/* SPI1 at /8 -> 64MHz/8 = 8 MHz  (well below 20 MHz limit after CPLOCK) */
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
  if (HAL_UART_Init(&huart2)                                         != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart2)                             != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* CS (PB0): output, idle HIGH */
  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);
  g.Pin   = DW1000_SS_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW1000_SS_PORT, &g);

  /* RSTn (PA8): starts as input Hi-Z — DW1000_HardReset() reconfigures */
  g.Pin  = DW1000_RST_PIN;
  g.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_RST_PORT, &g);

  /* IRQ (PA9): input */
  g.Pin  = DW1000_IRQ_PIN;
  g.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(DW1000_IRQ_PORT, &g);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
