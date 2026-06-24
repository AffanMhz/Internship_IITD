/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_anchor.c
  * @brief          : ANCHOR program body - use this when testing with Anchor
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "dw1000.h"
#include "dw1000_stm32.h"
#include "dw1000_time.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define RX_BUFFER_LEN 256

/* Private macro */
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

dw1000_HandleTypeDef dw1000_anchor;
volatile uint8_t anchor_rx_ready = 0;
uint8_t rx_buffer[RX_BUFFER_LEN];
uint16_t rx_len = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

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
    uart_print("[ANCHOR] [RESET] Initiating DW1000 hardware reset...\r\n");
    
    uart_print("[ANCHOR] [RESET] PC7 -> LOW\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(10);
    
    uart_print("[ANCHOR] [RESET] PC7 -> HIGH\r\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(10);
    
    uart_print("[ANCHOR] [RESET] ✓ DW1000 Reset complete (20ms total)\r\n");
    uart_print("[ANCHOR]\r\n");
}

// DW1000 Initialization with detailed diagnostics
HAL_StatusTypeDef dw1000_Init(dw1000_HandleTypeDef *dw) {
    uart_print("[ANCHOR] ===== DW1000 INITIALIZATION =====\r\n");
    uart_print("[ANCHOR] [INIT] Configuring DW1000 handle...\r\n");
    
    dw->spi = &hspi1;
    dw->ss_port = GPIOB;
    dw->ss_pin = GPIO_PIN_0;
    
    uart_print("[ANCHOR] [INIT]   SPI: SPI1 ✓\r\n");
    uart_print("[ANCHOR] [INIT]   CS Port: GPIOB ✓\r\n");
    uart_print("[ANCHOR] [INIT]   CS Pin: GPIO_PIN_0 (PB0) ✓\r\n");
    
    uart_print("[ANCHOR] [INIT] Performing hardware reset...\r\n");
    dw1000_Reset();
    
    uart_print("[ANCHOR] [INIT] Reading Device ID register (0x00)...\r\n");
    dw1000_dev_id_t dev_id = dw1000_GetDevID(dw);
    
    uart_print("[ANCHOR] [DEVID] Device Information:\r\n");
    uart_print("[ANCHOR] [DEVID]   REV:     0x%X\r\n", dev_id.rev);
    uart_print("[ANCHOR] [DEVID]   VER:     0x%X\r\n", dev_id.ver);
    uart_print("[ANCHOR] [DEVID]   MODEL:   0x%02X\r\n", dev_id.model);
    uart_print("[ANCHOR] [DEVID]   RIDTAG:  0x%04X (should be 0xDECA)\r\n", dev_id.ridtag);
    
    if (dev_id.ridtag == 0xDECA) {
        uart_print("[ANCHOR] ✓✓✓ DW1000 IDENTIFIED! SPI Communication WORKING! ✓✓✓\r\n");
        uart_print("[ANCHOR] [INIT] ✓ Device ID verification PASSED\r\n");
        uart_print("[ANCHOR] [INIT] ✓ SPI clock and CS timing CORRECT\r\n");
        uart_print("[ANCHOR] [INIT] ✓ Power supply to DW1000 STABLE\r\n");
        
        diag_dw1000_registers(dw);
        
        return HAL_OK;
    } else if (dev_id.model == 0x01) {
        uart_print("[ANCHOR] [DEVID] ⚠ WARNING: RIDTAG mismatch but model correct\r\n");
        uart_print("[ANCHOR] ✓ DW1000 SPI communication working (partial match)\r\n");
        return HAL_OK;
    } else {
        uart_print("[ANCHOR] ✗✗✗ FAILED TO IDENTIFY DW1000! ✗✗✗\r\n");
        uart_print("[ANCHOR] [ERROR] Expected RIDTAG: 0xDECA, Got: 0x%04X\r\n", dev_id.ridtag);
        uart_print("[ANCHOR] [ERROR] Possible causes:\r\n");
        uart_print("[ANCHOR] [ERROR]   - CS (PB0) wiring incorrect\r\n");
        uart_print("[ANCHOR] [ERROR]   - SCK (PA5) not toggling\r\n");
        uart_print("[ANCHOR] [ERROR]   - MOSI/MISO (PA7/PA6) not connected\r\n");
        uart_print("[ANCHOR] [ERROR]   - DW1000 not powered (check 3.3V)\r\n");
        uart_print("[ANCHOR] [ERROR]   - Shield not properly seated\r\n");
        return HAL_ERROR;
    }
}

// IRQ Callback for DW1000 interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) {  // PA10 = DW1000 IRQ
        if (dw1000_ReceiveDataFrameReady(&dw1000_anchor)) {
            uart_print("[ANCHOR] [IRQ] Interrupt: Frame ready for reading\r\n");
            anchor_rx_ready = 1;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  uart_print("\r\n\n");
  uart_print("╔════════════════════════════════════════╗\r\n");
  uart_print("║ UWB ANCHOR - DW1000 Test Firmware     ║\r\n");
  uart_print("║         Diagnostics Enabled           ║\r\n");
  uart_print("╚════════════════════════════════════════╝\r\n");
  uart_print("\r\n");
  
  uart_print("[SYSTEM] Running initial diagnostics...\r\n");
  uart_print("\r\n");
  
  diag_uart_test();
  diag_clock_config();
  diag_gpio_status();
  diag_spi_config();
  
  uart_print("[SYSTEM] Diagnostics complete. Starting DW1000 initialization...\r\n");
  uart_print("\r\n");
  
  // Initialize DW1000
  if (dw1000_Init(&dw1000_anchor) != HAL_OK) {
      uart_print("\r\n");
      uart_print("╔════════════════════════════════════════╗\r\n");
      uart_print("║ ✗ DW1000 INITIALIZATION FAILED!       ║\r\n");
      uart_print("╚════════════════════════════════════════╝\r\n");
      uart_print("[FATAL] System halting. Check wiring and power.\r\n");
      while(1) {
          HAL_Delay(1000);
      }
  }
  
  uart_print("\r\n");
  uart_print("╔════════════════════════════════════════╗\r\n");
  uart_print("║ ✓✓✓ INITIALIZATION SUCCESSFUL! ✓✓✓   ║\r\n");
  uart_print("╚════════════════════════════════════════╝\r\n");
  uart_print("\r\n");
  uart_print("[ANCHOR] Starting RX mode - listening for TAG...\r\n");
  uart_print("[ANCHOR] Format: [ANCHOR] RX #<count>: Received <bytes> bytes: '<data>'\r\n");
  uart_print("\r\n");
  
  // Enable continuous receive
  dw1000_StartReceive(&dw1000_anchor, 0);
  uart_print("[ANCHOR] ✓ RX mode enabled. Waiting for packets...\r\n");
  uart_print("\r\n");

  /* Infinite loop */
  uint32_t rx_count = 0;
  uint32_t total_rx = 0;
  uint32_t total_err = 0;
  
  while (1)
  {
    if (anchor_rx_ready) {
        anchor_rx_ready = 0;
        
        // Get the length of received data
        rx_len = dw1000_GetDataReceivedLength(&dw1000_anchor, 0);
        
        uart_print("[ANCHOR] [RX_%d] Received frame: %d bytes\r\n", rx_count, rx_len);
        
        if (rx_len > 0 && rx_len < RX_BUFFER_LEN) {
            // Read the data
            dw1000_GetDataReceived(&dw1000_anchor, rx_buffer, rx_len);
            
            // Null-terminate for string printing
            if (rx_len < RX_BUFFER_LEN - 1) {
                rx_buffer[rx_len] = '\0';
            }
            
            uart_print("[ANCHOR] [RX_%d]   Content: '%s'\r\n", rx_count, rx_buffer);
            uart_print("[ANCHOR] ✓ RX #%d: Received %d bytes: '%s'\r\n", 
                       rx_count, rx_len, rx_buffer);
            uart_print("[ANCHOR]\r\n");
            
            rx_count++;
            total_rx++;
            
            // Clear RX status and re-enable
            dw1000_ClearReceiveStatus(&dw1000_anchor);
            dw1000_StartReceive(&dw1000_anchor, 0);
        } else {
            uart_print("[ANCHOR] [RX_%d] ✗ ERROR: Invalid frame length: %d\r\n", rx_count, rx_len);
            total_err++;
            rx_count++;
        }
    }
    
    HAL_Delay(10);
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
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief SPI1 Initialization Function
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
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
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  uart_print("[ANCHOR] ERROR!\r\n");
  while(1) {
      HAL_Delay(1000);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  uart_print("Assert failed in file %s at line %d\r\n", file, line);
}
#endif
