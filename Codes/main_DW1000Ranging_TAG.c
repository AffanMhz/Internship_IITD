/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_DW1000Ranging_TAG.c
  * @brief          : DW1000 Simple Two-Way Ranging - TAG role
  *                    NUCLEO-G070RB
  ******************************************************************************
  * Pin map (per user wiring):
  *   DWM1000 SPICLK  (pin 20) -> PA5  (D13, SPI1_SCK)
  *   DWM1000 SPIMISO (pin 19) -> PA6  (D12, SPI1_MISO)
  *   DWM1000 SPIMOSI (pin 18) -> PA7  (D11, SPI1_MOSI)
  *   DWM1000 SPICSn  (pin 17) -> PB0  (D10, SS / Chip Select, manual GPIO)
  *   DWM1000 IRQ     (pin 22) -> PA9  (D8, input, polled)
  *   DWM1000 RSTn    (pin 3)  -> PA8  (D7, output, active low reset)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
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
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void DW1000_HardReset(dw1000_HandleTypeDef *dw1000);
static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw1000);
static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw1000);
static void UART_Print(const char *str);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
 * Simple Two-Way Ranging (SS-TWR) frame definitions - must match ANCHOR
 *
 * Function codes:
 *   0x20 = POLL      (Tag -> Anchor)
 *   0x21 = RESPONSE  (Anchor -> Tag)
 *   0x22 = FINAL     (Tag -> Anchor)  -- carries t_round (40-bit, bytes 1..5, little-endian)
 */
#define FC_POLL       0x20
#define FC_RESPONSE   0x21
#define FC_FINAL      0x22

#define DW1000_SS_PIN   GPIO_PIN_0   // PB0  (D10) - SPI CS, software controlled
#define DW1000_SS_PORT  GPIOB

#define DW1000_RST_PIN  GPIO_PIN_8   // PA8 (D7) - RSTn
#define DW1000_RST_PORT GPIOA

#define DW1000_IRQ_PIN  GPIO_PIN_9   // PA9 (D8) - IRQ, polled as input
#define DW1000_IRQ_PORT GPIOA

static dw1000_HandleTypeDef dw1000;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  char msg[80];

  dw1000_timestamp_t t_poll_tx = 0;
  dw1000_timestamp_t t_resp_rx = 0;
  dw1000_timestamp_t t_final_tx = 0;
  dw1000_timestamp_t t_round = 0;

  uint8_t frame_received;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Fill in DW1000 handle with SPI bus + CS pin (per user wiring) */
  dw1000.spi    = &hspi1;
  dw1000.ss_port = DW1000_SS_PORT;
  dw1000.ss_pin  = DW1000_SS_PIN;

  /* Hold CS idle high */
  HAL_GPIO_WritePin(DW1000_SS_PORT, DW1000_SS_PIN, GPIO_PIN_SET);

  /* Hardware reset the DW1000 */
  DW1000_HardReset(&dw1000);

  /* Read device ID to confirm SPI link is alive */
  dw1000_dev_id_t dev_id = dw1000_GetDevID(&dw1000);
  sprintf(msg, "\r\n[TAG] DEV_ID model=0x%02X ver=%u rev=%u ridtag=0x%04X\r\n",
          dev_id.model, dev_id.ver, dev_id.rev, dev_id.ridtag);
  UART_Print(msg);

  /* Configure PAN ID / short address (tag = 0x0002) */
  dw1000_pan_addr_t pan_addr;
  pan_addr.pan_id    = 0xDECA;
  pan_addr.short_addr = 0x0002;
  dw1000_SetPanAddress(&dw1000, &pan_addr);

  /* Set channel configuration (Channel 5, 16 MHz PRF, default preamble) - must match anchor */
  dw1000_channel_t chan;
  chan.tx_chan     = 5;
  chan.rx_chan     = 5;
  chan.prf         = DW1000_PRF_16MHZ;
  chan.tx_preamble = 0x4; // 1024 symbols code
  chan.rx_preamble = 0x4;
  dw1000_SetChannel(&dw1000, &chan);

  /* Clear any pending status flags before starting */
  dw1000_ClearAllStatus(&dw1000);

  UART_Print("[TAG] Initialized. Starting ranging cycles...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ---- Step 1: Send POLL to Anchor --------------------------------------- */
    dw1000_ClearTransmitStatus(&dw1000);
    tx_buf[0] = FC_POLL;
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 1, 1);
    dw1000_StartTransmit(&dw1000, 1, 1);

    uint8_t sent = 0;
    for (uint32_t i = 0; i < 2000000; i++)
    {
      uint8_t sys_status[DW1000_SYS_STATUS_LEN];
      dw1000_ReadData(&dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
      if (dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXFRS))
      {
        sent = 1;
        break;
      }
    }

    if (!sent)
    {
      HAL_Delay(500);
      continue;
    }

    t_poll_tx = dw1000_GetTxTimestamp(&dw1000);
    dw1000_ClearTransmitStatus(&dw1000);

    /* ---- Step 2: Wait for RESPONSE from Anchor ------------------------------ */
    dw1000_ClearReceiveStatus(&dw1000);
    dw1000_StartReceive(&dw1000, 1);

    frame_received = 0;
    for (uint32_t i = 0; i < 2000000; i++)
    {
      if (dw1000_ReceiveDataFrameReady(&dw1000))
      {
        frame_received = 1;
        break;
      }
    }

    if (!frame_received)
    {
      UART_Print("[TAG] Timeout waiting for RESPONSE\r\n");
      HAL_Delay(500);
      continue;
    }

    uint16_t len = dw1000_GetDataReceivedLength(&dw1000, 1);
    if (len == 0 || len > sizeof(rx_buf))
    {
      dw1000_ClearReceiveStatus(&dw1000);
      HAL_Delay(500);
      continue;
    }

    dw1000_GetDataReceived(&dw1000, rx_buf, len);
    dw1000_ClearReceiveStatus(&dw1000);

    if (rx_buf[0] != FC_RESPONSE)
    {
      HAL_Delay(500);
      continue;
    }

    t_resp_rx = dw1000_GetRxTimestamp(&dw1000);

    /* ---- Step 3: Send FINAL to Anchor ---------------------------------------
     *
     * The DW1000's own TX timestamp for THIS frame is only known AFTER it is
     * sent, so it cannot be embedded inside its own payload without delayed
     * transmission support (not used by this minimal polled driver).
     *
     * Workaround: the FINAL frame carries t_round from the PREVIOUS ranging
     * cycle (t_final_tx_prev - t_poll_tx_prev), piggy-backed on this cycle's
     * transmission. The anchor pairs each FINAL with the POLL/RESPONSE pair
     * that immediately preceded it; on the very first cycle t_round is 0 and
     * the anchor simply discards that first range estimate.
     *
     * This keeps every frame a true single TX/RX exchange while still letting
     * the anchor compute time-of-flight using only timestamps both sides
     * actually possess. */

    dw1000_ClearTransmitStatus(&dw1000);
    tx_buf[0] = FC_FINAL;
    for (int b = 0; b < 5; b++)
    {
      tx_buf[1 + b] = (uint8_t)(t_round >> (8 * b)); /* t_round from PREVIOUS cycle */
    }
    dw1000_SetDataToTransmit(&dw1000, tx_buf, 6, 1);
    dw1000_StartTransmit(&dw1000, 6, 1);

    sent = 0;
    for (uint32_t i = 0; i < 2000000; i++)
    {
      uint8_t sys_status[DW1000_SYS_STATUS_LEN];
      dw1000_ReadData(&dw1000, DW1000_SYS_STATUS, sys_status, DW1000_SYS_STATUS_LEN);
      if (dw1000_IsSet(sys_status, DW1000_SYS_STATUS_TXFRS))
      {
        sent = 1;
        break;
      }
    }

    if (!sent)
    {
      HAL_Delay(500);
      continue;
    }

    t_final_tx = dw1000_GetTxTimestamp(&dw1000);
    dw1000_ClearTransmitStatus(&dw1000);

    /* t_round for THIS cycle (t_final_tx - t_poll_tx); will be sent piggy-backed
     * in NEXT cycle's FINAL frame */
    t_round = t_final_tx - t_poll_tx;

    (void)t_resp_rx;

    sprintf(msg, "[TAG] cycle done, t_round=%.4f us (sent next cycle)\r\n",
            DW1000_Time_TimestampToMicroseconds(t_round));
    UART_Print(msg);

    HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

static void UART_Print(const char *str)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/* Hardware reset sequence for DW1000 RSTn (active low) */
static void DW1000_HardReset(dw1000_HandleTypeDef *dw1000)
{
  (void)dw1000;
  HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
}

/* Read 40-bit TX timestamp from TX_TIME register (offset 0 = TX_STAMP) */
static dw1000_timestamp_t dw1000_GetTxTimestamp(dw1000_HandleTypeDef *dw1000)
{
  uint8_t buffer[5];
  dw1000_ReadData(dw1000, DW1000_TX_TIME, buffer, 5);
  return ((dw1000_timestamp_t)buffer[4] << 32) | ((dw1000_timestamp_t)buffer[3] << 24) |
         ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] << 8)  |
          (dw1000_timestamp_t)buffer[0];
}

/* Read 40-bit RX timestamp from RX_TIME register (offset 0 = RX_STAMP) */
static dw1000_timestamp_t dw1000_GetRxTimestamp(dw1000_HandleTypeDef *dw1000)
{
  uint8_t buffer[5];
  dw1000_ReadData(dw1000, DW1000_RX_TIME, buffer, 5);
  return ((dw1000_timestamp_t)buffer[4] << 32) | ((dw1000_timestamp_t)buffer[3] << 24) |
         ((dw1000_timestamp_t)buffer[2] << 16) | ((dw1000_timestamp_t)buffer[1] << 8)  |
          (dw1000_timestamp_t)buffer[0];
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  *
  * PB0  (D10) -> DW1000 SPI CS (SPICSn), output, idle HIGH
  * PA8  (D7)  -> DW1000 RSTn,            output, idle HIGH (not asserted)
  * PA9  (D8)  -> DW1000 IRQ,             input (polled, not configured as EXTI here)
  * PA5/PA6/PA7 are configured by HAL_SPI_Init via MX_SPI1_Init (alternate function)
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Set CS (PB0) idle HIGH before configuring as output to avoid a glitch */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /* Set RST (PA8) idle HIGH (DW1000 not in reset) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /* Configure GPIO pin : PB0 (SPI CS, manual control) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PA8 (RSTn) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pin : PA9 (IRQ, input) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
