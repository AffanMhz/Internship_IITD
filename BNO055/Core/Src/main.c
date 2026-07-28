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
#include "bno055.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  STATE_STANDING = 0,
  STATE_KNEELING,
  STATE_PRONE,
} WorkerState_t;

#define PHASE1_RMS_WINDOW_SAMPLES (100U)

typedef struct {
  float samples[PHASE1_RMS_WINDOW_SAMPLES];
  uint16_t index;
  uint16_t count;
  float sum;
  float sum_sq;
  float mean;
  float variance;
  float std_dev;
  float max;
  float min;
} Phase1Window_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---- Phase 3: Lever-arm vector (neck base -> IMU/UWB tag on helmet) ----
// MEASURE these on your own helmet mount, in metres, in the IMU's body frame.
// Placeholder assumes the tag sits ~20 cm straight above the neck pivot.
#define LEVER_ARM_X (0.00f)
#define LEVER_ARM_Y (0.00f)
#define LEVER_ARM_Z (0.20f)

// ---- Phase 2: Posture classification thresholds ----
#define KNEEL_PITCH_THRESH_DEG (-45.0f) // pitch below this = candidate kneel/bend
#define KNEEL_HOLD_MS          (1500U)  // must hold for this long (debounce)
#define PRONE_ANGLE_THRESH_DEG (75.0f)  // |pitch| or |roll| beyond this = prone/fallen
#define G_MS2                  (9.80665f)
#define NEAR_1G_BAND_MS2       (1.5f)   // tolerance band around 1g for "still standing/kneeling"
#define FREEFALL_THRESH_MS2    (2.0f)   // |a| below this = freefall signature
#define IMPACT_THRESH_MS2      (25.0f)  // |a| above this = impact signature

// ---- Phase 2: Virtual-Z biomechanical invariants (metres) ----
#define VIRTUAL_Z_STANDING (1.70f)
#define VIRTUAL_Z_KNEELING (1.00f)
#define VIRTUAL_Z_PRONE    (0.20f)

// ---- Phase 1: IMU motion-intensity analysis ----
#define PHASE1_SAMPLE_PERIOD_MS (10U)
#define PHASE1_CSV_PERIOD_MS (50U)
#define PHASE1_REPORT_PERIOD_MS (250U)
#define PHASE1_DEFAULT_GRAVITY_MS2 (9.80665f)

// ---- Phase 4: Simple pedometer (step counting + velocity estimate) ----
// Peak/trough thresholds on the low-pass-filtered accel magnitude. A step is
// armed when the filtered magnitude rises above the high threshold (the
// push-off spike) and confirmed when it subsequently falls below the low
// threshold (the trough between steps). Tune these against your own logged
// CSV data -- they depend on mount location/orientation.
#define PEDOMETER_STEP_THRESH_HIGH_MS2 (11.5f)
#define PEDOMETER_STEP_THRESH_LOW_MS2  (8.5f)
// Fastest / slowest plausible step-to-step interval, used to reject noise
// (bounces counted as extra steps) and to decay velocity to zero once the
// person has stopped moving.
#define PEDOMETER_MIN_STEP_INTERVAL_MS (250U)
#define PEDOMETER_MAX_STEP_INTERVAL_MS (2000U)
// Exponential low-pass filter coefficient applied to the raw accel magnitude
// before peak detection (0 < alpha <= 1; lower = smoother/slower).
#define PEDOMETER_LPF_ALPHA             (0.25f)
// Weinberg dynamic stride-length model: stride = K * (peak-trough swing)^0.25.
// K is a per-person gait constant -- 0.50 is a reasonable adult default but
// should be calibrated against a known walking distance for real accuracy.
#define PEDOMETER_STRIDE_K              (0.50f)

// ---- Phase 5: velocity -> PWM mapping (drives a magnetic induction device) ----
// Timer: TIM3 CH1 on PA6 (Nucleo-G070RB Arduino header D12, AF1).
// VERIFY this pin/AF against your own CubeMX pinout before flashing --
// board silkscreens and AF tables vary by package/revision.
// PCLK1 = 64 MHz per the SystemClock_Config in this file (PLL 16MHz*8/2, no APB
// prescaler), so PSC/ARR below give a 500 Hz carrier. The compare register is
// then scaled from 1000 (minimal activity / near-zero response) up to 2000
// (highest plausible walking speed for an average human using their legs).
#define PWM_TIM_PSC              (63U)    // 64 MHz / (63+1)  = 1 MHz timer tick
#define PWM_TIM_ARR              (2000U)  // 1 MHz / (2000) = 500 Hz PWM carrier
#define PWM_DUTY_MIN_STEPS       (1000U)  // near-zero / low-motion output
#define PWM_DUTY_MAX_STEPS       (2000U)  // highest plausible human leg speed
// Walking speed (m/s) that should map to the top of the PWM command range.
// Tune this against your induction driver's actual control range -- this is a
// placeholder based on a brisk adult walking pace, not a measured value.
#define PWM_VELOCITY_MAX_MPS     (2.5f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bno055_t imu;
volatile uint8_t csv_header_printed = 0;

Phase1Window_t phase1_acc_mag_window;
Phase1Window_t phase1_motion_window;
Phase1Window_t phase1_rms_window;
float phase1_gravity_estimate = PHASE1_DEFAULT_GRAVITY_MS2;
float phase1_current_acc_mag = 0.0f;
float phase1_current_motion_mag = 0.0f;
float phase1_current_rms = 0.0f;
float phase1_idle_threshold = 0.08f;
float phase1_walk_threshold = 0.35f;
float phase1_run_threshold = 0.85f;
uint32_t phase1_last_sample_ms = 0;
uint32_t phase1_last_csv_ms = 0;
uint32_t phase1_last_report_ms = 0;

// Global declarations for STM32CubeIDE Live Watch/Debugging

WorkerState_t current_state = STATE_STANDING;
float virtual_z = 1.70f;
float comp_offsetX = 0.0f;
float comp_offsetY = 0.0f;
float svm = 0.0f;

// ---- Phase 4: pedometer state ----
float ped_filtered_mag = 9.80665f;   // low-pass-filtered accel magnitude, seeded at 1g
uint8_t ped_armed = 0;               // 1 while waiting for the falling edge that confirms a step
float ped_peak_mag = 0.0f;
float ped_trough_mag = 0.0f;
uint32_t ped_last_step_ms = 0;
uint32_t ped_step_count = 0;
float ped_stride_length_m = 0.0f;
float ped_velocity_mps = 0.0f;       // instantaneous walking speed estimate
float ped_velocity_from_cadence_mps = 0.0f; // explicit steps/s -> m/s estimate
float ped_distance_m = 0.0f;         // cumulative distance walked
float ped_cadence_spm = 0.0f;        // steps per minute
float ped_cadence_sps = 0.0f;        // steps per second (same cadence, different units)
uint8_t ped_step_detected = 0;       // pulses 1 for the sample a step was confirmed on

// ---- Phase 5: PWM output state ----
TIM_HandleTypeDef htim3;              // drives the magnetic induction device on PA6/TIM3_CH1
uint32_t current_pwm_ccr = 0; // Add this line

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void classify_posture(const bno055_euler_t *euler, float svm_val);
static float virtual_z_for_state(WorkerState_t state);
static void compute_lever_arm(const bno055_vec4_t *q, float *offX, float *offY);
static void phase1_window_reset(Phase1Window_t *window);
static void phase1_window_update(Phase1Window_t *window, float value);
static uint8_t pedometer_update(float raw_mag, uint32_t now_ms);
static void MX_TIM3_PWM_Init(void);
static void pwm_set_from_velocity(float velocity_mps);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void phase1_window_reset(Phase1Window_t *window)
{
  memset(window, 0, sizeof(*window));
  window->max = 0.0f;
  window->min = 0.0f;
}

static void phase1_window_update(Phase1Window_t *window, float value)
{
  if (window->count < PHASE1_RMS_WINDOW_SAMPLES)
  {
    window->samples[window->index] = value;
    window->count++;
  }
  else
  {
    float old_value = window->samples[window->index];
    window->sum -= old_value;
    window->sum_sq -= old_value * old_value;
    window->samples[window->index] = value;
  }

  window->sum += value;
  window->sum_sq += value * value;
  window->index = (window->index + 1U) % PHASE1_RMS_WINDOW_SAMPLES;

  if (window->count > 0U)
  {
    window->mean = window->sum / (float)window->count;
    float variance = (window->sum_sq / (float)window->count) - (window->mean * window->mean);
    if (variance < 0.0f)
    {
      variance = 0.0f;
    }
    window->variance = variance;
    window->std_dev = sqrtf(variance);
  }

  float max_val = -1000000.0f;
  float min_val = 1000000.0f;
  for (uint16_t i = 0; i < window->count; i++)
  {
    float sample = window->samples[i];
    if (sample > max_val)
    {
      max_val = sample;
    }
    if (sample < min_val)
    {
      min_val = sample;
    }
  }

  window->max = max_val;
  window->min = min_val;
}

// Debounce timer state for the kneel/bend transition (S0 -> S1)
static uint32_t kneel_timer_start  = 0;
static uint8_t  kneel_timer_active = 0;

/**
 * @brief Phase 2: Posture classification ("Virtual-Z" trigger).
 *        Updates the global current_state using pitch/roll + a hold timer
 *        for S1 and an instantaneous check for S2 (prone/fallen).
 */
static void classify_posture(const bno055_euler_t *euler, float svm_val)
{
  // S2 - Prone/Fallen: extreme tilt, OR a freefall/impact acceleration signature.
  // This check is instantaneous (no debounce) since it's a safety-critical state.
  if (fabsf(euler->pitch) > PRONE_ANGLE_THRESH_DEG ||
      fabsf(euler->roll)  > PRONE_ANGLE_THRESH_DEG ||
      svm_val < FREEFALL_THRESH_MS2 ||
      svm_val > IMPACT_THRESH_MS2)
  {
    current_state = STATE_PRONE;
    kneel_timer_active = 0;
    return;
  }

  // S1 - Kneeling/Bending: pitch below threshold, sustained for KNEEL_HOLD_MS,
  // AND the accel magnitude stays near 1g (rules out a fall in progress).
  uint8_t near_1g = (svm_val > (G_MS2 - NEAR_1G_BAND_MS2)) &&
                    (svm_val < (G_MS2 + NEAR_1G_BAND_MS2));

  if (euler->pitch < KNEEL_PITCH_THRESH_DEG && near_1g)
  {
    if (!kneel_timer_active)
    {
      kneel_timer_active = 1;
      kneel_timer_start  = HAL_GetTick();
    }
    if ((HAL_GetTick() - kneel_timer_start) > KNEEL_HOLD_MS)
    {
      current_state = STATE_KNEELING;
    }
    // else: still within the debounce window, hold last classified state
  }
  else
  {
    kneel_timer_active = 0;
    current_state = STATE_STANDING;
  }
}

/**
 * @brief Maps a discrete posture state to its Virtual-Z biomechanical invariant.
 */
static float virtual_z_for_state(WorkerState_t state)
{
  switch (state)
  {
    case STATE_KNEELING: return VIRTUAL_Z_KNEELING;
    case STATE_PRONE:    return VIRTUAL_Z_PRONE;
    case STATE_STANDING:
    default:             return VIRTUAL_Z_STANDING;
  }
}

/**
 * @brief Phase 3: Lever-arm compensation.
 *        Rotates the fixed neck->tag lever-arm vector by the IMU's current
 *        orientation quaternion, producing the (x,y) planar displacement the
 *        tag has moved away from the torso due to head rotation alone.
 *        A host-side EKF should subtract (offX, offY) from the raw UWB
 *        tag coordinate to recover the stabilized torso position:
 *            p_torso = p_tag - R(q) * l_arm
 */
static void compute_lever_arm(const bno055_vec4_t *q, float *offX, float *offY)
{
  float w = q->w, x = q->x, y = q->y, z = q->z;

  // Rows 0 and 1 of the rotation matrix derived from the quaternion
  // (row 2 / Z is not needed here: Z is handled separately via Virtual-Z)
  float r00 = 1.0f - 2.0f * (y * y + z * z);
  float r01 = 2.0f * (x * y - w * z);
  float r02 = 2.0f * (x * z + w * y);
  float r10 = 2.0f * (x * y + w * z);
  float r11 = 1.0f - 2.0f * (x * x + z * z);
  float r12 = 2.0f * (y * z - w * x);

  *offX = r00 * LEVER_ARM_X + r01 * LEVER_ARM_Y + r02 * LEVER_ARM_Z;
  *offY = r10 * LEVER_ARM_X + r11 * LEVER_ARM_Y + r12 * LEVER_ARM_Z;
}

/**
 * @brief Phase 4: Simple peak/trough pedometer with a dynamic (Weinberg)
 *        stride-length model, driven off the raw accel-magnitude signal.
 *
 *        1) Low-pass filters the raw accel magnitude to smooth sensor noise.
 *        2) Arms when the filtered signal rises above a high threshold (the
 *           push-off spike of a footstep) and confirms the step when it
 *           subsequently falls below a low threshold (the trough between
 *           steps), with a minimum-interval debounce to reject bounce.
 *        3) On each confirmed step, estimates stride length from the
 *           peak-to-trough swing (harder swing -> longer stride) and divides
 *           by the step interval to get an instantaneous walking velocity.
 *        4) If no step has landed for PEDOMETER_MAX_STEP_INTERVAL_MS, the
 *           velocity/cadence decay to zero so the display doesn't get stuck
 *           showing a stale walking speed once the person has stopped.
 *
 * @param  raw_mag Raw accel magnitude ||accel|| (m/s^2) for this sample --
 *                 callers that already compute svm should pass it straight
 *                 in to avoid a redundant sqrt.
 * @param  now_ms  Current tick count (HAL_GetTick()).
 * @retval 1 on the sample a step was confirmed, 0 otherwise.
 */
static uint8_t pedometer_update(float raw_mag, uint32_t now_ms)
{
  // Smooth the raw signal so single-sample spikes don't false-trigger.
  ped_filtered_mag += PEDOMETER_LPF_ALPHA * (raw_mag - ped_filtered_mag);

  uint8_t step_detected = 0;

  if (!ped_armed)
  {
    // Waiting for the rising edge (push-off) of the next step.
    if (ped_filtered_mag > PEDOMETER_STEP_THRESH_HIGH_MS2)
    {
      ped_armed = 1U;
      ped_peak_mag = ped_filtered_mag;
      ped_trough_mag = ped_filtered_mag;
    }
  }
  else
  {
    if (ped_filtered_mag > ped_peak_mag)   ped_peak_mag = ped_filtered_mag;
    if (ped_filtered_mag < ped_trough_mag) ped_trough_mag = ped_filtered_mag;

    if (ped_filtered_mag < PEDOMETER_STEP_THRESH_LOW_MS2)
    {
      // Falling edge confirms the step, provided it's not just bounce off
      // the previous one.
      uint32_t interval_ms = now_ms - ped_last_step_ms;
      if (ped_last_step_ms == 0U || interval_ms >= PEDOMETER_MIN_STEP_INTERVAL_MS)
      {
        ped_step_count++;
        step_detected = 1U;

        // Weinberg dynamic stride estimate: stride = K * swing^0.25.
        float swing = ped_peak_mag - ped_trough_mag;
        if (swing < 0.0f) swing = 0.0f;
        ped_stride_length_m = PEDOMETER_STRIDE_K * sqrtf(sqrtf(swing));

        if (ped_last_step_ms != 0U && interval_ms > 0U && interval_ms < PEDOMETER_MAX_STEP_INTERVAL_MS)
        {
          float interval_s = (float)interval_ms / 1000.0f;
          ped_cadence_spm = 60000.0f / (float)interval_ms;
          ped_cadence_sps = 1000.0f / (float)interval_ms;

          // Velocity from cadence is the simple relationship:
          // steps/second * stride_length_in_meters = metres/second.
          ped_velocity_from_cadence_mps = ped_stride_length_m * ped_cadence_sps;
          ped_velocity_mps = ped_velocity_from_cadence_mps;

          // The old interval-based form gives the same result mathematically:
          // ped_stride_length_m / interval_s == ped_stride_length_m * cadence_sps.
          (void)interval_s;
        }
        else
        {
          // First step ever, or the gap was too long to trust a velocity from.
          ped_velocity_mps = 0.0f;
          ped_velocity_from_cadence_mps = 0.0f;
          ped_cadence_spm = 0.0f;
          ped_cadence_sps = 0.0f;
        }

        ped_distance_m += ped_stride_length_m;
        ped_last_step_ms = now_ms;
      }

      ped_armed = 0U;
    }
  }

  // No step for too long -> the person has stopped, decay speed/cadence to 0.
  if (ped_last_step_ms != 0U && (now_ms - ped_last_step_ms) > PEDOMETER_MAX_STEP_INTERVAL_MS)
  {
    ped_velocity_mps = 0.0f;
    ped_cadence_spm = 0.0f;
    ped_cadence_sps = 0.0f;
  }

  return step_detected;
}

/**
 * @brief Phase 5: One-time TIM3 PWM peripheral setup (PA6 / TIM3_CH1).
 *        Kept in USER CODE so it survives CubeMX regeneration -- if you add
 *        TIM3 in the .ioc file instead, delete this and use the generated
 *        MX_TIM3_Init() the normal way.
 */
static void MX_TIM3_PWM_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PWM_TIM_PSC;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_TIM_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;                       // start at 0% duty (stationary)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Phase 5: Maps walking velocity to a PWM duty cycle driving the
 *        induction device -- faster steps -> higher duty cycle -> stronger
 *        drive signal. velocity_mps is already stride_length / step_interval,
 *        which is mathematically the same thing as stride_length * cadence_sps,
 *        so no separate steps/second-based recomputation is needed here.
 * @param velocity_mps Current walking speed estimate (m/s), e.g. ped_velocity_mps.
 */
static void pwm_set_from_velocity(float velocity_mps)
{
  float v = velocity_mps;
  if (v < 0.0f)
  {
    v = 0.0f;
  }
  if (v > PWM_VELOCITY_MAX_MPS)
  {
    v = PWM_VELOCITY_MAX_MPS;
  }

  float normalized = v / PWM_VELOCITY_MAX_MPS;
  uint32_t ccr = (uint32_t)lroundf((float)PWM_DUTY_MIN_STEPS +
                                    normalized * (float)(PWM_DUTY_MAX_STEPS - PWM_DUTY_MIN_STEPS));

  if (ccr < PWM_DUTY_MIN_STEPS)
  {
    ccr = PWM_DUTY_MIN_STEPS;
  }
  if (ccr > PWM_DUTY_MAX_STEPS)
  {
    ccr = PWM_DUTY_MAX_STEPS;
  }

  current_pwm_ccr = ccr; // Save the value globally here!
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Initialize BNO055 sensor
  imu = bno055_new();
  imu.i2c = &hi2c1;
  imu.addr = BNO_ADDR;
  imu.mode = BNO_MODE_NDOF;

  // Verify BNO055 presence by reading chip ID
  uint8_t chip_id = 0;
  HAL_I2C_Master_Transmit(&hi2c1, (BNO_ADDR << 1), (uint8_t *)&chip_id, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, (BNO_ADDR << 1), &chip_id, 1, HAL_MAX_DELAY);

  // Send verification status to UART
  char msg[100];
  if (chip_id == BNO_DEF_CHIP_ID) {
    sprintf(msg, "BNO055 detected! Chip ID: 0x%02X\r\n", chip_id);
  } else {
    sprintf(msg, "BNO055 NOT found! Expected 0x%02X, got 0x%02X\r\n", BNO_DEF_CHIP_ID, chip_id);
  }
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  // Initialize the sensor if detected
  if (chip_id == BNO_DEF_CHIP_ID) {
    error_bno init_err = bno055_init(&imu);
    if (init_err != BNO_OK) {
      sprintf(msg, "BNO055 initialization failed: %s\r\n", bno055_err_str(init_err));
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      Error_Handler();
    }
    sprintf(msg, "BNO055 initialized successfully!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  } else {
    Error_Handler();
  }

  phase1_window_reset(&phase1_acc_mag_window);
  phase1_window_reset(&phase1_motion_window);
  phase1_window_reset(&phase1_rms_window);

  // Phase 5: bring up the PWM output that will drive the induction device,
  // starting at 0% duty until the first velocity estimate arrives.
  MX_TIM3_PWM_Init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((HAL_GetTick() - phase1_last_sample_ms) < PHASE1_SAMPLE_PERIOD_MS)
    {
      HAL_Delay(1U);
      continue;
    }
    phase1_last_sample_ms = HAL_GetTick();

    // Print CSV header once
    if (!csv_header_printed) {
      char header[] = "roll,pitch,yaw,qw,qx,qy,qz,accx,accy,accz,gyrox,gyroy,gyroz,svm,state,z_v,lever_offx,lever_offy,steps,stride_m,velocity_mps,cadence_spm,cadence_sps,distance_m,step_flag\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)header, strlen(header), HAL_MAX_DELAY);
      csv_header_printed = 1;
    }

    // Read sensor data
    bno055_euler_t euler;
    bno055_vec3_t accel, gyro;
    bno055_vec4_t quat;

    if (imu.euler(&imu, &euler) == BNO_OK &&
        imu.acc(&imu, &accel) == BNO_OK &&
        imu.gyro(&imu, &gyro) == BNO_OK &&
        imu.quaternion(&imu, &quat) == BNO_OK) {

      // Phase 2: posture classification (Virtual-Z trigger)
      svm = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
      classify_posture(&euler, svm);
      virtual_z = virtual_z_for_state(current_state);

      // Phase 3: lever-arm compensation (helmet rotation vs torso position)
      compute_lever_arm(&quat, &comp_offsetX, &comp_offsetY);

      // Phase 4: pedometer (step count + walking velocity estimate).
      // Reuses svm (computed just above) as the raw accel-magnitude input so
      // we don't recompute the same sqrt twice per loop iteration.
      ped_step_detected = pedometer_update(svm, HAL_GetTick());

      // Phase 5: push the current velocity estimate out to the induction
      // device as a PWM duty cycle (0 m/s -> 0%, PWM_VELOCITY_MAX_MPS -> 100%).
      pwm_set_from_velocity(ped_velocity_mps);

      // Phase 1: compute motion-intensity metrics using a 100-sample window.
      phase1_current_acc_mag = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
      phase1_window_update(&phase1_acc_mag_window, phase1_current_acc_mag);
      phase1_gravity_estimate = phase1_acc_mag_window.mean;
      phase1_current_motion_mag = fabsf(phase1_current_acc_mag - phase1_gravity_estimate);
      phase1_window_update(&phase1_motion_window, phase1_current_motion_mag);
      phase1_current_rms = sqrtf(phase1_motion_window.sum_sq / (float)phase1_motion_window.count);
      phase1_window_update(&phase1_rms_window, phase1_current_rms);

      phase1_idle_threshold = fmaxf(0.08f, phase1_rms_window.mean + 2.0f * phase1_rms_window.std_dev);
      phase1_walk_threshold = fmaxf(0.35f, phase1_idle_threshold + 0.20f);
      phase1_run_threshold = fmaxf(0.85f, phase1_walk_threshold + 0.50f);

      const char *phase1_state = "standing";
      if (phase1_current_rms > phase1_run_threshold)
      {
        phase1_state = "running";
      }
      else if (phase1_current_rms > phase1_walk_threshold)
      {
        phase1_state = "walking";
      }
      else if (phase1_current_rms > phase1_idle_threshold)
      {
        phase1_state = "slow";
      }

      if ((HAL_GetTick() - phase1_last_report_ms) >= PHASE1_REPORT_PERIOD_MS)
      {
        char report[220];
        int report_len = snprintf(report, sizeof(report),
                                  "phase1 acc_mag=%.3f gravity=%.3f motion=%.3f moving_avg=%.3f rms=%.3f mean_rms=%.3f max_rms=%.3f min_rms=%.3f std=%.3f var=%.3f state=%s thr=%.3f/%.3f/%.3f\r\n",
                                  phase1_current_acc_mag,
                                  phase1_gravity_estimate,
                                  phase1_current_motion_mag,
                                  phase1_motion_window.mean,
                                  phase1_current_rms,
                                  phase1_rms_window.mean,
                                  phase1_rms_window.max,
                                  phase1_rms_window.min,
                                  phase1_rms_window.std_dev,
                                  phase1_rms_window.variance,
                                  phase1_state,
                                  phase1_idle_threshold,
                                  phase1_walk_threshold,
                                  phase1_run_threshold);
        HAL_UART_Transmit(&huart2, (uint8_t *)report, report_len, HAL_MAX_DELAY);
        phase1_last_report_ms = HAL_GetTick();
      }

      if ((HAL_GetTick() - phase1_last_csv_ms) >= PHASE1_CSV_PERIOD_MS)
            {
              // Format and send CSV data including the new PWM value
              char data[370];
              int len = sprintf(data, "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.4f,%.4f,%lu,%.3f,%.3f,%.2f,%.2f,%.3f,%d,%lu\r\n",
                               euler.roll, euler.pitch, euler.yaw,
                               quat.w, quat.x, quat.y, quat.z,
                               accel.x, accel.y, accel.z,
                               gyro.x, gyro.y, gyro.z,
                               svm, (int)current_state, virtual_z,
                               comp_offsetX, comp_offsetY,
                               (unsigned long)ped_step_count, ped_stride_length_m,
                               ped_velocity_mps, ped_cadence_spm, ped_cadence_sps, ped_distance_m,
                               (int)ped_step_detected,
                               (unsigned long)current_pwm_ccr); // <-- Added PWM here

              HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY);
              phase1_last_csv_ms = HAL_GetTick();
            }
    }
  }
  /* USER CODE END 3 */
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B17DB5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Printf retargeting to UART for debugging
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

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
