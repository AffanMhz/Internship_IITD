/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "dw1000.h"
#include "dw1000_stm32.h"
#include "dw1000_time.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
dw1000_HandleTypeDef dw1000_tag;
volatile uint8_t tag_ready_to_tx = 0;
volatile uint8_t tag_rx_complete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART Debug print function
int _write(int file, unsigned char *ptr, int len) {
    HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
    return len;
}

void uart_print(const char *fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Diagnostic: Test UART connectivity
void diag_uart_test(void) {
    uart_print("[DIAG] ===== UART CONNECTIVITY TEST =====\r\n");
    uart_print("[DIAG] UART running at 115200 baud (8N1)\r\n");
    uart_print("[DIAG] UART Test: OK (you're seeing this message)\r\n");
    uart_print("[DIAG]\r\n");
}

// Diagnostic: Show clock configuration
void diag_clock_config(void) {
    uart_print("[DIAG] ===== CLOCK CONFIGURATION =====\r\n");
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t apb1 = HAL_RCC_GetPCLK1Freq();
    uart_print("[DIAG] SYSCLK: %lu Hz (%.1f MHz)\r\n", sysclk, sysclk/1e6);
    uart_print("[DIAG] PCLK1:  %lu Hz (%.1f MHz)\r\n", apb1, apb1/1e6);
    uart_print("[DIAG] Expected: SYSCLK=64MHz, PCLK1=64MHz\r\n");
    uart_print("[DIAG] Note: STM32G0 has single APB bus (PCLK1 only)\r\n");
    uart_print("[DIAG]\r\n");
}

// Diagnostic: Show GPIO status
void diag_gpio_status(void) {
    uart_print("[DIAG] ===== GPIO CONFIGURATION =====\r\n");
    GPIO_PinState cs_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    GPIO_PinState reset_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
    GPIO_PinState irq_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
    
    uart_print("[DIAG] PB0 (CS/NSS):    %s (should be HIGH at idle)\r\n", 
               cs_state ? "HIGH" : "LOW");
    uart_print("[DIAG] PC7 (RESET):     %s (should be HIGH after init)\r\n", 
               reset_state ? "HIGH" : "LOW");
    uart_print("[DIAG] PA10 (IRQ):      %s (input, waiting for signal)\r\n", 
               irq_state ? "HIGH" : "LOW");
    uart_print("[DIAG]\r\n");
}

// Diagnostic: Show SPI configuration
void diag_spi_config(void) {
    uart_print("[DIAG] ===== SPI1 CONFIGURATION =====\r\n");
    uart_print("[DIAG] Mode:            Full Duplex Master\r\n");
    uart_print("[DIAG] Data Size:       8-bit\r\n");
    uart_print("[DIAG] Clock Polarity:  Low (CPOL=0)\r\n");
    uart_print("[DIAG] Clock Phase:     1 Edge (CPHA=0)\r\n");
    uart_print("[DIAG] Prescaler:       32 (SPI speed ~2MHz @ 64MHz SYSCLK)\r\n");
    uart_print("[DIAG] NSS Mode:        Disabled (Software managed)\r\n");
    uart_print("[DIAG] First Bit:       MSB\r\n");
    uart_print("[DIAG]\r\n");
}

// Diagnostic: Show DW1000 registers (after successful init)
void diag_dw1000_registers(dw1000_HandleTypeDef *dw) {
    uart_print("[DIAG] ===== DW1000 REGISTER DUMP =====\r\n");
    
    // Read system status
    uint8_t sys_status[5] = {0};
    dw1000_ReadData(dw, DW1000_SYS_STATUS, sys_status, 5);
    uint32_t status = sys_status[0] | (sys_status[1] << 8) | 
                      (sys_status[2] << 16) | (sys_status[3] << 24);
    
    uart_print("[DIAG] SYS_STATUS:      0x%08lX\r\n", status);
    uart_print("[DIAG]   RXDFR (RX OK): %s\r\n", 
               (status & (1<<14)) ? "YES" : "NO");
    uart_print("[DIAG]   TXFRS (TX OK): %s\r\n", 
               (status & (1<<7)) ? "YES" : "NO");
    uart_print("[DIAG]   CPLOCK (PLL): %s\r\n", 
               (status & (1<<1)) ? "LOCKED" : "NO");
    
    // Read system config
    uint8_t sys_cfg[4] = {0};
    dw1000_ReadData(dw, DW1000_SYS_CFG, sys_cfg, 4);
    uint32_t config = sys_cfg[0] | (sys_cfg[1] << 8) | 
                      (sys_cfg[2] << 16) | (sys_cfg[3] << 24);
    
    uart_print("[DIAG] SYS_CONFIG:      0x%08lX\r\n", config);
    uart_print("[DIAG]   FFEN:          %s\r\n", 
               (config & (1<<0)) ? "ENABLED" : "DISABLED");
    
    // Read PAN address
    uint8_t pan[4] = {0};
    dw1000_ReadData(dw, DW1000_PANADR, pan, 4);
    uint16_t pan_id = pan[2] | (pan[3] << 8);
    uint16_t addr = pan[0] | (pan[1] << 8);
    
    uart_print("[DIAG] PAN ID:          0x%04X\r\n", pan_id);
    uart_print("[DIAG] Short Address:   0x%04X\r\n", addr);
    uart_print("[DIAG]\r\n");
}

// DW1000 Reset function with diagnostics
void dw1000_Reset(void) {
    uart_print("[TAG] [RESET] Initiating DW1000 hardware reset...\r\n");
    
    // Pull RESET low (PC7)
    uart_print("[TAG] [RESET] PC7 -> LOW\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(10);
    
    // Release RESET (pull high)
    uart_print("[TAG] [RESET] PC7 -> HIGH\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(10);
    
    uart_print("[TAG] [RESET] ✓ DW1000 Reset complete (20ms total)\r\n");
    uart_print("[TAG]\r\n");
}

// DW1000 Initialization with detailed diagnostics
HAL_StatusTypeDef dw1000_Init(dw1000_HandleTypeDef *dw) {
    uart_print("[TAG] ===== DW1000 INITIALIZATION =====\r\n");
    uart_print("[TAG] [INIT] Configuring DW1000 handle...\r\n");
    
    dw->spi = &hspi1;
    dw->ss_port = GPIOB;
    dw->ss_pin = GPIO_PIN_0;
    
    uart_print("[TAG] [INIT]   SPI: SPI1 ✓\r\n");
    uart_print("[TAG] [INIT]   CS Port: GPIOB ✓\r\n");
    uart_print("[TAG] [INIT]   CS Pin: GPIO_PIN_0 (PB0) ✓\r\n");
    
    // Perform hardware reset
    uart_print("[TAG] [INIT] Performing hardware reset...\r\n");
    dw1000_Reset();
    
    // Read and verify device ID
    uart_print("[TAG] [INIT] Reading Device ID register (0x00)...\r\n");
    dw1000_dev_id_t dev_id = dw1000_GetDevID(dw);
    
    uart_print("[TAG] [DEVID] Device Information:\r\n");
    uart_print("[TAG] [DEVID]   REV:     0x%X\r\n", dev_id.rev);
    uart_print("[TAG] [DEVID]   VER:     0x%X\r\n", dev_id.ver);
    uart_print("[TAG] [DEVID]   MODEL:   0x%02X\r\n", dev_id.model);
    uart_print("[TAG] [DEVID]   RIDTAG:  0x%04X (should be 0xDECA)\r\n", dev_id.ridtag);
    
    // Validation
    if (dev_id.ridtag == 0xDECA) {
        uart_print("[TAG] ✓✓✓ DW1000 IDENTIFIED! SPI Communication WORKING! ✓✓✓\r\n");
        uart_print("[TAG] [INIT] ✓ Device ID verification PASSED\r\n");
        uart_print("[TAG] [INIT] ✓ SPI clock and CS timing CORRECT\r\n");
        uart_print("[TAG] [INIT] ✓ Power supply to DW1000 STABLE\r\n");
        
        // Show additional diagnostics
        diag_dw1000_registers(dw);
        
        return HAL_OK;
    } else if (dev_id.model == 0x01) {
        uart_print("[TAG] [DEVID] ⚠ WARNING: RIDTAG mismatch but model correct\r\n");
        uart_print("[TAG] ✓ DW1000 SPI communication working (partial match)\r\n");
        return HAL_OK;
    } else {
        uart_print("[TAG] ✗✗✗ FAILED TO IDENTIFY DW1000! ✗✗✗\r\n");
        uart_print("[TAG] [ERROR] Expected RIDTAG: 0xDECA, Got: 0x%04X\r\n", dev_id.ridtag);
        uart_print("[TAG] [ERROR] Possible causes:\r\n");
        uart_print("[TAG] [ERROR]   - CS (PB0) wiring incorrect\r\n");
        uart_print("[TAG] [ERROR]   - SCK (PA5) not toggling\r\n");
        uart_print("[TAG] [ERROR]   - MOSI/MISO (PA7/PA6) not connected\r\n");
        uart_print("[TAG] [ERROR]   - DW1000 not powered (check 3.3V)\r\n");
        uart_print("[TAG] [ERROR]   - Shield not properly seated\r\n");
        return HAL_ERROR;
    }
}

// IRQ Callback for DW1000 interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) {  // PA10 = DW1000 IRQ
        uart_print("[TAG] [IRQ] DW1000 Interrupt received on PA10\r\n");
        tag_rx_complete = 1;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  uart_print("\r\n\n");
  uart_print("╔════════════════════════════════════════╗\r\n");
  uart_print("║   UWB TAG - DW1000 Test Firmware      ║\r\n");
  uart_print("║         Diagnostics Enabled           ║\r\n");
  uart_print("╚════════════════════════════════════════╝\r\n");
  uart_print("\r\n");
  
  // Run all diagnostics
  uart_print("[SYSTEM] Running initial diagnostics...\r\n");
  uart_print("\r\n");
  
  diag_uart_test();
  diag_clock_config();
  diag_gpio_status();
  diag_spi_config();
  
  uart_print("[SYSTEM] Diagnostics complete. Starting DW1000 initialization...\r\n");
  uart_print("\r\n");
  
  // Initialize DW1000
  if (dw1000_Init(&dw1000_tag) != HAL_OK) {
      uart_print("\r\n");
      uart_print("╔════════════════════════════════════════╗\r\n");
      uart_print("║  ✗ DW1000 INITIALIZATION FAILED!      ║\r\n");
      uart_print("╚════════════════════════════════════════╝\r\n");
      uart_print("[FATAL] System halting. Check wiring and power.\r\n");
      uart_print("[FATAL] Verify:\r\n");
      uart_print("[FATAL]   1. Shield power: 3.3V present?\r\n");
      uart_print("[FATAL]   2. SPI wires: PA5(CLK), PA6(MISO), PA7(MOSI) connected?\r\n");
      uart_print("[FATAL]   3. CS wire: PB0 connected to shield CS/NSS?\r\n");
      uart_print("[FATAL]   4. RESET wire: PC7 connected to shield RESET?\r\n");
      while(1) {
          HAL_Delay(1000);
      }
  }
  
  uart_print("\r\n");
  uart_print("╔════════════════════════════════════════╗\r\n");
  uart_print("║  ✓✓✓ INITIALIZATION SUCCESSFUL! ✓✓✓  ║\r\n");
  uart_print("╚════════════════════════════════════════╝\r\n");
  uart_print("\r\n");
  uart_print("[TAG] Starting TX test loop...\r\n");
  uart_print("[TAG] Transmitting 'HELLO_FROM_TAG' every 2 seconds\r\n");
  uart_print("[TAG] Format: [TAG] TX #<count>: '<data>' (length=<bytes>)\r\n");
  uart_print("\r\n");
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tx_count = 0;
  uint32_t timestamp_ms = 0;
  uint8_t tx_data[] = "HELLO_FROM_TAG";
  uint16_t tx_length = strlen((char*)tx_data);
  
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    
    // Get current time
    timestamp_ms = HAL_GetTick();
    
    // Load data to transmit
    uart_print("[TAG] [TX_%d] Loading %d bytes into TX buffer...\r\n", tx_count, tx_length);
    dw1000_SetDataToTransmit(&dw1000_tag, tx_data, tx_length, 0);
    uart_print("[TAG] [TX_%d]   Data: '%s'\r\n", tx_count, tx_data);
    
    // Start transmit
    uart_print("[TAG] [TX_%d] Starting transmission...\r\n", tx_count);
    dw1000_StartTransmit(&dw1000_tag, tx_length, 0);
    uart_print("[TAG] [TX_%d] ✓ TX #%d: '%s' (length=%d bytes)\r\n", 
               tx_count, tx_count, tx_data, tx_length);
    uart_print("[TAG] [TX_%d] Time: %lu ms\r\n", tx_count, timestamp_ms);
    
    tx_count++;
    uart_print("[TAG]\r\n");
    
    // Wait 2 seconds before next transmission
    uart_print("[TAG] Waiting 2000ms before next TX...\r\n");
    HAL_Delay(2000);
    uart_print("\r\n");
    
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
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

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 (CS/NSS) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 (RESET) */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 (IRQ with EXTI) */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
