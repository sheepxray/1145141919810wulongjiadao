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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "wulong_control.h"
#include "imu.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  K3_TEST_IDLE = 0,
  K3_TEST_CALIBRATING,
  K3_TEST_SPIN,
  K3_TEST_BRAKE,
  K3_TEST_HEADING_RETURN,
  K3_TEST_POSITION_CHECK,
  K3_TEST_POSITION_DRIVE,
  K3_TEST_FINAL_HEADING,
  K3_TEST_DONE,
  K3_TEST_ERROR
} K3_TestState;

#define K3_TRACE_CAPACITY 1000U

typedef struct {
  uint32_t t_ms;
  float yaw_mrad;
  float gx_dps;
  float gy_dps;
  float gz_dps;
  float bias_x_dps;
  float bias_y_dps;
  float bias_z_dps;
  float heading_error_mrad;
  float position_error_mm;
  float x_mm;
  float y_mm;
  float command_vx_mmps;
  float command_vy_mmps;
  float command_omega_mradps;
  int16_t target_omega_mradps;
  uint8_t state;
  uint8_t stationary;
  uint8_t active;
  uint8_t reserved;
} K3_GyroTraceEntry;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static TaskHandle_t control_task_handle;
static TaskHandle_t comms_task_handle;
static TaskHandle_t strategy_task_handle;
static TaskHandle_t imu_task_handle;

/*
 * Motor bring-up test state.
 *
 * The normal command path is USART1 at 115200 (PC4=TX, PC5=RX):
 *   ARM, DISARM, S, E, ENC, Z, V, STAT, H
 *   A+/A-/B+/B-/C+/C-/D+/D-          quick test, 10% for 800 ms
 *   A+ 1500                          quick test, 10% for 1500 ms
 *   A+ 15 1500                       quick test, 15% for 1500 ms
 *   RUN A + 15 1500                  explicit test, requires ARM
 *
 * These volatile globals are also intended for Keil Watch:
 * set g_armed=1, then set g_test_trigger=1 after filling the other fields.
 */
volatile uint8_t  g_armed = 0;
volatile uint8_t  g_test_trigger = 0;
volatile uint8_t  g_test_motor = 0;       /* 0=A, 1=B, 2=C, 3=D */
volatile int8_t   g_test_dir = 1;         /* +1 forward, -1 reverse */
volatile uint16_t g_test_duty_permille = 100;
volatile uint16_t g_test_duration_ms = 800;
volatile uint8_t  g_estop_request = 0;
volatile uint8_t  g_k3_test_request = 0;
volatile uint8_t  g_speed_tune_request = 0;
volatile uint8_t  g_k2_test_request = 0;

static volatile uint8_t  active_motor = 0;
static volatile int8_t   active_dir = 0;
static volatile uint32_t active_until_ms = 0;
static volatile uint8_t  drive_mode_active = 0;
static volatile uint8_t  all_full_active = 0;
static volatile uint32_t all_full_until_ms = 0;
static volatile uint8_t  k1_seq_active = 0;
static volatile uint8_t  k2_ramp_active = 0;
static volatile uint8_t  turn_right_active = 0;
static volatile uint8_t  k3_test_active = 0;

static K3_TestState k3_state = K3_TEST_IDLE;
static uint32_t k3_state_ms;
static uint32_t k3_random_state = 0x6D2B79F5UL;
static float    k3_target_omega_mradps;
static float    k3_spin_cmd_mradps;
static float    k3_spin_rate_integral;
static float    k3_position_error_mm;
static float    k3_heading_error_mrad;
static uint8_t  k3_cal_seen;
static uint8_t  k3_heading_retry;
static uint8_t  k3_heading_recoveries;
static float    k3_heading_best_abs_mrad;
static uint32_t k3_heading_progress_ms;
static uint8_t  k3_position_corrections;
static float    k3_position_best_mm;
static uint32_t k3_position_progress_ms;
static uint8_t  k3_head_cycle_phase;
static uint8_t  k3_head_attempts;
static uint32_t k3_head_phase_ms;
static float    k3_yaw_rate_filtered;
static float    k3_yaw_cmd_last;
static const char *k3_last_result = "idle";
static K3_GyroTraceEntry k3_gyro_trace[K3_TRACE_CAPACITY];
static volatile uint16_t k3_gyro_trace_count;
static uint32_t k3_gyro_trace_last_ms;

#define K3_POSITION_STOP_MM          2.0f
#define K3_POSITION_ACCEPT_MM        4.0f
#define K3_POSITION_LIMIT_MM         300.0f
#define K3_POSITION_MAX_SPEED_MMPS   80.0f
#define K3_POSITION_GAIN             1.5f
#define K3_POSITION_RATE_GAIN        0.35f
#define K3_POSITION_MAX_CORRECTIONS  5U
#define K3_POSITION_TIMEOUT_MS       8000U
#define K3_POSITION_STALL_MS         2000U
#define K3_SPIN_RAMP_MS              350U
#define K3_ACTIVE_BRAKE_MS           350U
#define K3_SPIN_RATE_FF              1.55f
#define K3_SPIN_RATE_KP              0.55f
#define K3_SPIN_RATE_KI              1.20f
#define K3_SPIN_RATE_INTEGRAL_LIMIT  1200.0f
#define K3_SPIN_COMMAND_LIMIT        1800.0f
#define K3_HEADING_STALL_MS          1800U
#define K3_HEADING_MAX_RECOVERIES    2U

#define SPEED_TUNE_PHASE_COUNT     3U
#define SPEED_TUNE_SETTLE_MS       700U
#define SPEED_TUNE_MEASURE_MS      800U
#define SPEED_TUNE_PHASE_MS        (SPEED_TUNE_SETTLE_MS + SPEED_TUNE_MEASURE_MS)

static volatile uint8_t speed_tune_active;
static volatile uint8_t speed_tune_phase;
static volatile uint8_t speed_tune_done;
static uint32_t speed_tune_phase_ms;
static uint32_t speed_tune_sample_count;
static float speed_tune_sum[WULONG_WHEEL_COUNT];
static float speed_tune_sum_sq[WULONG_WHEEL_COUNT];
static float speed_tune_duty_sum[WULONG_WHEEL_COUNT];
static float speed_tune_min[WULONG_WHEEL_COUNT];
static float speed_tune_max[WULONG_WHEEL_COUNT];
static float speed_tune_rise_ms[SPEED_TUNE_PHASE_COUNT][WULONG_WHEEL_COUNT];
static float speed_tune_mean[SPEED_TUNE_PHASE_COUNT][WULONG_WHEEL_COUNT];
static float speed_tune_ripple[SPEED_TUNE_PHASE_COUNT][WULONG_WHEEL_COUNT];
static float speed_tune_std[SPEED_TUNE_PHASE_COUNT][WULONG_WHEEL_COUNT];
static float speed_tune_duty[SPEED_TUNE_PHASE_COUNT][WULONG_WHEEL_COUNT];
static const float speed_tune_targets[SPEED_TUNE_PHASE_COUNT] = {
  100.0f, 150.0f, 200.0f
};

/* When set, the control task runs the closed-loop chassis stack
 * (Wulong_ControlTick) instead of the bring-up test state machines. */
static volatile uint8_t  g_ctrl_console_active = 0;

/* Vision approach mode.  K230 sends normalized observations over USART2 and
 * this task owns all speed limiting, target-loss handling and stopping. */
static volatile uint8_t  g_vision_approach_active = 0;

#define K230_RX_BUF_SIZE          256U
#define K230_FRAME_BUF_SIZE       160U
#define K230_MAX_PAYLOAD          128U
#define K230_MSG_HELLO            0x05U
#define K230_MSG_APPROACH_TARGET  0x07U

#define VISION_TIMEOUT_MS         200U
#define VISION_MIN_CONF           35U
#define VISION_STOP_SIZE_PM       580
#define VISION_DEADBAND_PM        50
#define VISION_ALIGN_FULL_PM      500
#define VISION_FORWARD_ERR_PM     200
#define VISION_MAX_VX_MMPS        100.0f
#define VISION_MAX_OMEGA_MRADPS   350.0f

static volatile uint8_t s_k230_rx_buf[K230_RX_BUF_SIZE];
static volatile uint16_t s_k230_rx_head;
static volatile uint16_t s_k230_rx_tail;
static volatile uint32_t s_k230_rx_count;
static uint8_t s_k230_rx_byte;

static uint8_t s_k230_frame_buf[K230_FRAME_BUF_SIZE];
static uint16_t s_k230_frame_len;
static volatile int16_t s_vision_err_pm;
static volatile int16_t s_vision_size_pm;
static volatile uint8_t s_vision_conf;
static volatile uint8_t s_vision_valid;
static volatile uint32_t s_vision_last_ms;
static volatile uint32_t s_vision_frame_count;
static volatile uint32_t s_vision_crc_errors;

/*
 * Telemetry mode: run the IMU at 1 kHz, keep the motors disarmed, and print
 * one whitespace-separated numeric line at a fixed rate.  This is meant for
 * a serial oscilloscope / VOFA+ / SerialPlot channel while the wheels are off
 * the ground, so the gyro can be observed while the chassis is turned by hand.
 */
static volatile uint8_t  g_imu_telemetry = 0;
static volatile uint16_t g_imu_telemetry_div = 10U;
static volatile uint8_t  g_imu_report = 0;
static volatile uint16_t g_imu_report_div = 2000U;
static volatile uint8_t  g_imu_auto_cal_suppressed = 0U;
/* Set while the comms task is parsing/answering a command, so telemetry lines
 * never split a command reply on the shared USART1. */
static volatile uint8_t  g_cli_busy = 0;

/*
 * USART1 receive ring buffer.
 *
 * The previous implementation polled HAL_UART_Receive() once per 2 ms, but a
 * byte at 115200 baud arrives every ~87 us and USART1 FIFO is disabled, so all
 * but the first byte of every command was lost.  The RX interrupt now pushes
 * bytes into this buffer and CommsTask drains it.
 */
#define CLI_RX_BUF_SIZE 128U
static volatile uint8_t  s_cli_rx_buf[CLI_RX_BUF_SIZE];
static volatile uint16_t s_cli_rx_head;
static volatile uint16_t s_cli_rx_tail;
static volatile uint32_t s_cli_rx_count;
static uint8_t           s_cli_rx_byte;
/* Byte echo, off by default; toggle with the ECHO command when needed. */
static volatile uint8_t  g_cli_echo = 0U;


/*
 * Physical wheel map, confirmed by the user on 2026-10-01:
 *   A = front-left,  B = rear-left,  C = rear-right,  D = front-right
 *
 * Do NOT assume A/B/C/D are FL/FR/RL/RR.  All motion patterns below are
 * written in terms of these physical positions so that rearranging the wiring
 * only requires changing this table.
 */
enum {
  WHEEL_FL = 0,   /* A */
  WHEEL_RL = 1,   /* B */
  WHEEL_RR = 2,   /* C */
  WHEEL_FR = 3    /* D */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ControlTask(void *argument);
static void CommsTask(void *argument);
static void StrategyTask(void *argument);
static void ImuTask(void *argument);

static void MotorTest_Init(void);
static void MotorTest_Arm(void);
static void MotorTest_Disarm(void);
static void MotorTest_SetOutput(uint8_t motor, int8_t dir, uint16_t permille,
                                uint8_t allow_full_duty);
static void MotorTest_Start(uint8_t motor, int8_t dir, uint16_t permille, uint32_t duration_ms);
static void MotorTest_RunFullA(void);
static void MotorTest_DriveAll(int8_t dir);
static void MotorTest_RunAllFull(void);
static void MotorTest_RunAllFullStep(void);
static void MotorTest_K1SeqStart(void);
static void MotorTest_K1SeqStep(void);
static void MotorTest_K1SeqApply(uint8_t leg);
static void MotorTest_K2RampStart(void);
static void MotorTest_K2RampStep(void);
static void MotorTest_TurnRightStart(void);
static void MotorTest_TurnRightStep(void);
static void MotorTest_K3Start(void);
static void MotorTest_K3Step(void);
static void MotorTest_K3TraceSample(uint32_t now_ms);
static void MotorTest_K3Finish(const char *result);
static float MotorTest_K3NextOmega(void);
static float MotorTest_K3HeadingCommand(float error_mrad,
                                        float measured_omega_mradps,
                                        float max_rate_mradps);
static void MotorTest_K3HeadingCycleReset(void);
static void MotorTest_K3ApplyPositionReturn(const Wulong_Pose *pose,
                                            float omega_cmd_mradps);
static uint8_t MotorTest_K3HeadingCycle(float error_mrad,
                                        float measured_omega_mradps,
                                        uint32_t now_ms,
                                        float max_rate_mradps,
                                        uint8_t max_attempts,
                                        float error_tol_mrad,
                                        float rate_tol_mradps);
static float MotorTest_WrapMrad(float angle_mrad);
static float MotorTest_ClampFloat(float value, float min_value, float max_value);
static void MotorTest_SpeedTuneStart(void);
static void MotorTest_SpeedTuneStep(void);
static void MotorTest_SpeedTuneFinish(const char *result);
static void MotorTest_KeyScan(void);
static void MotorTest_StopOutputs(void);
static void MotorTest_RunSingle(uint8_t motor, int8_t dir);
static void MotorTest_AutoStart(void);
static void MotorTest_AutoCancel(void);
static void MotorTest_AutoStep(void);
static uint8_t MotorTest_AutoIsActive(void);
static int32_t MotorTest_ReadEncoder(uint8_t motor);
static void MotorTest_ResetEncoders(void);
static uint32_t MotorTest_ReadBatteryMv(void);
static void MotorTest_HandleCommand(char *line);
static uint8_t MotorTest_DipSwitchMode(void);
static void Uart_SendText(const char *text);
static void Uart_SendU32(uint32_t value);
static void Uart_SendS32(int32_t value);
static void Uart_SendMotor(uint8_t motor);
static void Uart_SendImuReport(void);
static const char *SkipSpaces(const char *p);
static uint8_t ParseU32(const char **p, uint32_t *value);
static uint8_t ParseFloat(const char **p, float *value);
static uint8_t ParseMotor(const char **p, uint8_t *motor);
static void TrimCommand(char *line);
static uint8_t IsTelemetrySafeCommand(const char *line);
static uint8_t CliGetByte(uint8_t *byte);
static void CliRxStart(void);
static uint8_t K230GetByte(uint8_t *byte);
static void K230RxStart(void);
static void K230ParserFeed(uint8_t byte);
static void VisionApproachStart(void);
static void VisionApproachStep(uint32_t now_ms);
static void VisionApproachStop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM20_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
/* USER CODE BEGIN 2 */
  /*
   * Print the firmware tag as early as possible.  Previously the first
   * UART output happened only after MotorTest_Init() / CliRxStart() /
   * Imu_Init(), so a hang anywhere in those three looked like a totally dead
   * board (no output at all).  The staged markers below make the failure
   * point visible instead.
   */
  Uart_SendText("\r\n" WULONG_FW_TAG "\r\n");
  Uart_SendText("BOOT: motor\r\n");
  MotorTest_Init();
  Uart_SendText("BOOT: rx\r\n");
  CliRxStart();
  K230RxStart();
  Uart_SendText("BOOT: imu\r\n");
  (void)Imu_Init();
  Uart_SendText("IMU ready=");
  Uart_SendU32(Imu_IsReady());
  Uart_SendText(Imu_IsReady() ? " (ICM-20602 on SPI2)\r\n" : " (NOT FOUND)\r\n");
  Uart_SendText("WULONG motor test ready\r\n");
  Uart_SendText("Default: STBY low, all PWM=0. Type H for help.\r\n");

  xTaskCreate(ControlTask, "control", 256, NULL, 4, &control_task_handle);
  xTaskCreate(CommsTask, "comms", 256, NULL, 3, &comms_task_handle);
  xTaskCreate(StrategyTask, "strategy", 384, NULL, 2, &strategy_task_handle);
  xTaskCreate(ImuTask, "imu", 256, NULL, 5, &imu_task_handle);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* The scheduler should never return. */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage.
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void MotorTest_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  /* M-board K1 is PD8, active-low, pulled up when released. */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  gpio_init.Pin = GPIO_PIN_8;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  /* M-board K2 is PD9, active-low, pulled up when released. */
  gpio_init.Pin = GPIO_PIN_9;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  /* M-board K3 is PD10, active-low, pulled up when released. */
  gpio_init.Pin = GPIO_PIN_10;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim20, TIM_CHANNEL_ALL);
}

static void MotorTest_Arm(void)
{
  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_SET);
  g_armed = 1;
}

static void MotorTest_Disarm(void)
{
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  MotorTest_StopOutputs();
  g_armed = 0;
}

static void MotorTest_StopOutputs(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

  HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin | CIN1_Pin | CIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, DIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, DIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_RESET);

  active_dir = 0;
  active_until_ms = 0;
  drive_mode_active = 0;
  all_full_active = 0;
  all_full_until_ms = 0;
}

static void MotorTest_SetOutput(uint8_t motor, int8_t dir, uint16_t permille,
                                uint8_t allow_full_duty)
{
  static const uint32_t channels[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
  };
  uint32_t pulse;
  uint8_t forward;

  if (motor > 3U || dir == 0) {
    return;
  }
  if (!allow_full_duty && permille > 300U) {
    permille = 300U;
  }
  if (permille > 1000U) {
    permille = 1000U;
  }
  pulse = ((uint32_t)permille * 16999U) / 1000U;

  /* A flipped wheel has both its direction lines inverted here and its feedback
   * negated in MotorTest_ReadEncoder - see the FLIP console command. */
  forward = (uint8_t)(dir > 0 ? 1U : 0U);
  if (Wulong_GetWheelFlip((Wulong_Wheel)motor)) {
    forward = (uint8_t)(forward ? 0U : 1U);
  }

  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_SET);

  switch (motor) {
  case 0:
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(GPIOE, BIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, BIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case 2:
    HAL_GPIO_WritePin(GPIOB, CIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, CIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  default:
    HAL_GPIO_WritePin(GPIOD, DIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, DIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channels[motor], pulse);
}

static void MotorTest_RunFullA(void)
{
  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  /* motor A, forward, 100% duty, 2000 ms hard timeout */
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 16999U);

  active_motor = 0U;
  active_dir = 1;
  active_until_ms = HAL_GetTick() + 2000U;

  Uart_SendText("A FULL FORWARD duty=1000 ms=2000\r\n");
}

static void MotorTest_DriveAll(int8_t dir)
{
  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  /*
   * Drive all four wheels in the same logical direction.
   * Motor C already has its logical direction inverted for its mirrored mount.
   * Keep the first driving test at 20% duty; use S/E for immediate stop.
   */
  MotorTest_SetOutput(0U, dir, 200U, 0U);
  MotorTest_SetOutput(1U, dir, 200U, 0U);
  MotorTest_SetOutput(2U, dir, 200U, 0U);
  MotorTest_SetOutput(3U, dir, 200U, 0U);

  drive_mode_active = 1U;
  active_motor = 0U;
  active_dir = dir;
  active_until_ms = 0U;

  Uart_SendText(dir > 0 ? "DRIVE FORWARD duty=200\r\n"
                        : "DRIVE REVERSE duty=200\r\n");
}

/*
 * All four wheels, logical forward, 100% duty for 2000 ms.
 *
 * Bench-only command.  It deliberately bypasses the normal 30% software
 * limit, so use it only with the wheels off the ground and the chassis fixed.
 * The 1 kHz control task stops and disarms at the deadline.
 */
#define ALL_FULL_DUTY_PERMILLE  1000U
#define ALL_FULL_DURATION_MS    2000U

static void MotorTest_RunAllFull(void)
{
  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  all_full_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  all_full_until_ms = HAL_GetTick() + ALL_FULL_DURATION_MS;
  all_full_active = 1U;
  active_motor = 0U;
  active_dir = 1;
  active_until_ms = all_full_until_ms;

  MotorTest_SetOutput(WHEEL_FL, 1, ALL_FULL_DUTY_PERMILLE, 1U);
  MotorTest_SetOutput(WHEEL_RL, 1, ALL_FULL_DUTY_PERMILLE, 1U);
  MotorTest_SetOutput(WHEEL_RR, 1, ALL_FULL_DUTY_PERMILLE, 1U);
  MotorTest_SetOutput(WHEEL_FR, 1, ALL_FULL_DUTY_PERMILLE, 1U);

  Uart_SendText("ALL FULL FORWARD duty=1000 ms=2000\r\n");
}

static void MotorTest_RunAllFullStep(void)
{
  if (!all_full_active) {
    return;
  }
  if ((int32_t)(HAL_GetTick() - all_full_until_ms) < 0) {
    return;
  }

  all_full_active = 0U;
  MotorTest_Disarm();
  Uart_SendText("ALL FULL DONE\r\n");
}

/*
 * K1 four-phase timed sequence, 2000 ms per phase:
 *
 *   phase 0  forward        A+ B+ C+ D+
 *   phase 1  strafe left    A- B+ C- D+
 *   phase 2  strafe right   A+ B- C+ D-
 *   phase 3  backward       A- B- C- D-
 *
 * Wheel map is the confirmed physical map A=FL, B=RL, C=RR, D=FR.
 * Patterns come from the mecanum inverse kinematics in docs/03 §3.1,
 * using vx = forward, vy = left, w = CCW:
 *
 *   FL(A) = vx - vy + k*w
 *   FR(D) = vx + vy - k*w
 *   RL(B) = vx + vy + k*w
 *   RR(C) = vx - vy - k*w
 *
 * The left-side +k*w pivot sign is the one verified on the assembled chassis
 * with the IMU.  The old open-loop document used the opposite pivot sign.
 *
 * WARNING: open-loop strafe can slip on the 3D-printed rollers.  K3 closes the
 * translation loop with odometry; this sequence remains a direct bring-up test.
 * If strafe runs the wrong way, swap the signs in phases 1 and 2; if the
 * chassis turns instead of sliding, the rollers are not gripping.
 */
#define K1_SEQ_DUTY_PERMILLE  200U
#define K1_SEQ_PHASE_MS       2000U
#define K1_SEQ_PHASE_COUNT    4U

static uint8_t  k1_seq_phase = 0U;
static uint32_t k1_seq_phase_start_ms = 0U;
static const char *k1_seq_phase_names[K1_SEQ_PHASE_COUNT] = {
  "FWD", "STRAFE_L", "STRAFE_R", "BWD"
};

static void MotorTest_K1SeqStart(void)
{
  if (k1_seq_active) {
    return;
  }

  MotorTest_AutoCancel();
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_ResetEncoders();
  MotorTest_Arm();

  k1_seq_phase = 0U;
  k1_seq_active = 1U;
  k1_seq_phase_start_ms = HAL_GetTick();
  MotorTest_K1SeqApply(0U);
  Uart_SendText("K1 SEQ START: FWD STRAFE_L STRAFE_R BWD (2s each)\r\n");
  Uart_SendText("K1 PHASE FWD\r\n");
}

static void MotorTest_K1SeqApply(uint8_t phase)
{
  switch (phase) {
  case 0U:
    /* forward: A+ B+ C+ D+ */
    MotorTest_SetOutput(WHEEL_FL,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RL,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RR,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_FR,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    break;
  case 1U:
    /* strafe left: A- B+ C- D+ */
    MotorTest_SetOutput(WHEEL_FL, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RL,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RR, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_FR,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    break;
  case 2U:
    /* strafe right: A+ B- C+ D- */
    MotorTest_SetOutput(WHEEL_FL,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RL, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RR,  1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_FR, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    break;
  default:
    /* backward: A- B- C- D- */
    MotorTest_SetOutput(WHEEL_FL, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RL, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_RR, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    MotorTest_SetOutput(WHEEL_FR, -1, K1_SEQ_DUTY_PERMILLE, 0U);
    break;
  }
}

/*
 * K2: straight-forward drive with a linear 0 -> 100 % PWM ramp.
 *
 * The full-duty bypass is intentional for this bench test and is safe only
 * with the wheels off the ground.  The ramp takes five seconds, then the
 * output holds 100 % for another ten seconds.
 */
#define K2_RAMP_FULL_DUTY_PERMILLE  1000U
#define K2_RAMP_UP_MS               5000U
#define K2_FULL_SPEED_HOLD_MS       10000U
#define K2_RUN_DURATION_MS          (K2_RAMP_UP_MS + K2_FULL_SPEED_HOLD_MS)
#define K2_RAMP_REPORT_INTERVAL_MS  1000U

static uint32_t k2_ramp_start_ms = 0U;
static uint32_t k2_ramp_last_report_ms = 0U;
static volatile uint16_t k2_ramp_last_permille = 0U;

static void MotorTest_K2RampStart(void)
{
  if (k1_seq_active || k2_ramp_active || turn_right_active) {
    return;
  }

  MotorTest_AutoCancel();
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  k2_ramp_active = 1U;
  k2_ramp_start_ms = HAL_GetTick();
  k2_ramp_last_report_ms = 0U;
  k2_ramp_last_permille = 0U;
  Uart_SendText("K2 FORWARD 0->1000 in 5000 ms, hold 10000 ms\r\n");
}

static void MotorTest_K2RampStep(void)
{
  uint32_t elapsed_ms;
  uint32_t permille;

  if (!k2_ramp_active) {
    return;
  }

  elapsed_ms = (uint32_t)(HAL_GetTick() - k2_ramp_start_ms);
  if (elapsed_ms >= K2_RUN_DURATION_MS) {
    k2_ramp_last_permille = K2_RAMP_FULL_DUTY_PERMILLE;
    Uart_SendText("K2 t_ms=15000 duty=1000\r\n");
    k2_ramp_active = 0U;
    MotorTest_StopOutputs();
    Uart_SendText("K2 FORWARD DONE\r\n");
    return;
  }

  if (elapsed_ms < K2_RAMP_UP_MS) {
    permille = (elapsed_ms * K2_RAMP_FULL_DUTY_PERMILLE) /
               K2_RAMP_UP_MS;
  } else {
    permille = K2_RAMP_FULL_DUTY_PERMILLE;
  }
  k2_ramp_last_permille = (uint16_t)permille;

  /* straight forward: all four logical wheel directions are positive */
  MotorTest_SetOutput(WHEEL_FL, 1, (uint16_t)permille, 1U);
  MotorTest_SetOutput(WHEEL_RL, 1, (uint16_t)permille, 1U);
  MotorTest_SetOutput(WHEEL_RR, 1, (uint16_t)permille, 1U);
  MotorTest_SetOutput(WHEEL_FR, 1, (uint16_t)permille, 1U);

  if ((elapsed_ms - k2_ramp_last_report_ms) >=
      K2_RAMP_REPORT_INTERVAL_MS) {
    k2_ramp_last_report_ms = elapsed_ms;
    Uart_SendText("K2 t_ms=");
    Uart_SendU32(elapsed_ms);
    Uart_SendText(" duty=");
    Uart_SendU32(permille);
    Uart_SendText("\r\n");
  }
}

/*
 * Turn right (clockwise) in place for 1000 ms:
 *   A+ B+ C- D-
 */
#define TURN_RIGHT_DUTY_PERMILLE  200U
#define TURN_RIGHT_DURATION_MS    1000U

static uint32_t turn_right_until_ms = 0U;

static void MotorTest_TurnRightStart(void)
{
  if (k1_seq_active || k2_ramp_active || turn_right_active) {
    return;
  }

  MotorTest_AutoCancel();
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  /* turn right (CW): A+ B+ C- D- */
  MotorTest_SetOutput(WHEEL_FL,  1, TURN_RIGHT_DUTY_PERMILLE, 0U);
  MotorTest_SetOutput(WHEEL_RL,  1, TURN_RIGHT_DUTY_PERMILLE, 0U);
  MotorTest_SetOutput(WHEEL_RR, -1, TURN_RIGHT_DUTY_PERMILLE, 0U);
  MotorTest_SetOutput(WHEEL_FR, -1, TURN_RIGHT_DUTY_PERMILLE, 0U);

  turn_right_active = 1U;
  turn_right_until_ms = HAL_GetTick() + TURN_RIGHT_DURATION_MS;
  Uart_SendText("TURN RIGHT duty=200 ms=1000\r\n");
}

static void MotorTest_TurnRightStep(void)
{
  if (!turn_right_active) {
    return;
  }

  if ((int32_t)(HAL_GetTick() - turn_right_until_ms) >= 0) {
    turn_right_active = 0U;
    MotorTest_StopOutputs();
    Uart_SendText("TURN RIGHT DONE\r\n");
  }
}

static float MotorTest_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static float MotorTest_WrapMrad(float angle_mrad)
{
  const float pi_mrad = 3141.59265f;
  float wrapped = atan2f(sinf(angle_mrad * 0.001f),
                         cosf(angle_mrad * 0.001f)) * 1000.0f;

  /*
   * At exactly 180 degrees, choose the direction that unwinds the sign of the
   * raw angle.  This makes the tie deterministic instead of letting roundf()
   * decide, which produced occasional +180/-180 ambiguity.
   */
  if (fabsf(wrapped) > (pi_mrad - 1.0f)) {
    wrapped = (angle_mrad >= 0.0f) ? (pi_mrad - 1.0f)
                                   : (-pi_mrad + 1.0f);
  }
  return wrapped;
}

static float MotorTest_K3HeadingCommand(float error_mrad,
                                        float measured_omega_mradps,
                                        float max_rate_mradps)
{
  float abs_error = fabsf(error_mrad);
  float kp;
  float kd;
  float omega;
  float rate_alpha = 0.001f / (0.001f + 0.040f);
  float delta_cmd;

  /*
   * Gain scheduling and rate feedback suppress overshoot.  Near zero the
   * controller becomes mostly a brake; this removes the limit cycle seen when
   * the angle loop fought the wheel-speed loop at the target.
   */
  if (abs_error > 250.0f) {
    kp = 4.0f;
    kd = 1.2f;
  } else if (abs_error > 60.0f) {
    kp = 3.0f;
    kd = 1.5f;
  } else {
    kp = 1.5f;
    kd = 1.8f;
  }

  k3_yaw_rate_filtered +=
      (measured_omega_mradps - k3_yaw_rate_filtered) * rate_alpha;
  omega = kp * error_mrad - kd * k3_yaw_rate_filtered;
  if ((abs_error > 4.0f) && (fabsf(k3_yaw_rate_filtered) < 20.0f) &&
      (fabsf(omega) < 60.0f)) {
    omega = (error_mrad > 0.0f) ? 60.0f : -60.0f;
  }

  omega = MotorTest_ClampFloat(omega, -max_rate_mradps, max_rate_mradps);

  delta_cmd = omega - k3_yaw_cmd_last;
  delta_cmd = MotorTest_ClampFloat(delta_cmd, -4.0f, 4.0f);
  k3_yaw_cmd_last += delta_cmd;
  return k3_yaw_cmd_last;
}

static void MotorTest_K3HeadingCycleReset(void)
{
  k3_head_cycle_phase = 0U;
  k3_head_attempts = 0U;
  k3_head_phase_ms = HAL_GetTick();
  k3_yaw_rate_filtered = 0.0f;
  k3_yaw_cmd_last = 0.0f;
}

/*
 * Translate directly back to the origin with the mecanum wheels while the
 * heading loop holds the calibrated direction.  The position error is rotated
 * from the field frame into the current body frame, then a small velocity PD
 * term supplies braking as the chassis approaches the origin.
 */
static void MotorTest_K3ApplyPositionReturn(const Wulong_Pose *pose,
                                            float omega_cmd_mradps)
{
  float error_x = -pose->x_mm;
  float error_y = -pose->y_mm;
  float theta_rad = pose->theta_mrad * 0.001f;
  float c = cosf(theta_rad);
  float s = sinf(theta_rad);
  float body_error_x = error_x * c + error_y * s;
  float body_error_y = -error_x * s + error_y * c;
  float distance_mm = sqrtf(error_x * error_x + error_y * error_y);
  float vx_meas;
  float vy_meas;
  float vx_cmd;
  float vy_cmd;
  float command_magnitude;
  float speed_limit;
  float heading_scale;
  float heading_error_mrad;

  if (distance_mm <= K3_POSITION_STOP_MM) {
    Wulong_SetBodyVelocityXY(0.0f, 0.0f, omega_cmd_mradps);
    return;
  }

  Wulong_GetMeasuredTranslationVelocity(&vx_meas, &vy_meas);

  /*
   * Return one body axis at a time.  A diagonal mecanum command combines
   * longitudinal and roller-side forces and slips more readily; cardinal
   * front/back or left/right moves are easier to reproduce on the floor.
   */
  if (fabsf(body_error_x) >= fabsf(body_error_y)) {
    vx_cmd = (K3_POSITION_GAIN * body_error_x) -
             (K3_POSITION_RATE_GAIN * vx_meas);
    vy_cmd = 0.0f;
    speed_limit = MotorTest_ClampFloat(fabsf(body_error_x) * 1.35f, 0.0f,
                                       K3_POSITION_MAX_SPEED_MMPS);
  } else {
    vx_cmd = 0.0f;
    vy_cmd = (K3_POSITION_GAIN * body_error_y) -
             (K3_POSITION_RATE_GAIN * vy_meas);
    speed_limit = MotorTest_ClampFloat(fabsf(body_error_y) * 1.35f, 0.0f,
                                       K3_POSITION_MAX_SPEED_MMPS);
  }

  command_magnitude = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
  if ((command_magnitude > speed_limit) && (command_magnitude > 0.01f)) {
    float scale = speed_limit / command_magnitude;
    vx_cmd *= scale;
    vy_cmd *= scale;
  }

  /*
   * If lateral slip or a wheel disturbance starts to rotate the chassis,
   * reduce translation so the heading loop can recover the axis before the
   * mecanum vector is applied at full strength.
   */
  heading_error_mrad = MotorTest_WrapMrad(-pose->theta_mrad);
  heading_scale = 1.0f -
      ((fabsf(heading_error_mrad) - 20.0f) / 140.0f);
  heading_scale = MotorTest_ClampFloat(heading_scale, 0.15f, 1.0f);
  vx_cmd *= heading_scale;
  vy_cmd *= heading_scale;

  Wulong_SetBodyVelocityXY(vx_cmd, vy_cmd, omega_cmd_mradps);
}

/*
 * Continuous-approach + coast + stationary-validation heading controller.
 *
 * Large errors use the reference project's continuous angle outer loop with
 * rate braking.  Once inside a narrow window, torque is removed and the actual
 * IMU heading is validated after the chassis settles.  A failed validation
 * starts another approach instead of continuously chasing the target.
 *
 * Return: 0 = busy, 1 = validated, 2 = attempts exhausted.
 */
static uint8_t MotorTest_K3HeadingCycle(float error_mrad,
                                        float measured_omega_mradps,
                                        uint32_t now_ms,
                                        float max_rate_mradps,
                                        uint8_t max_attempts,
                                        float error_tol_mrad,
                                        float rate_tol_mradps)
{
  uint32_t phase_elapsed_ms = (uint32_t)(now_ms - k3_head_phase_ms);
  float abs_error = fabsf(error_mrad);
  float omega_cmd;

  switch (k3_head_cycle_phase) {
  case 0U:  /* continuous angle approach with rate braking */
    if ((abs_error <= error_tol_mrad) &&
        (fabsf(measured_omega_mradps) <= 30.0f)) {
      Wulong_Stop();
      k3_head_cycle_phase = 1U;
      k3_head_phase_ms = now_ms;
      break;
    }

    omega_cmd = MotorTest_K3HeadingCommand(
        error_mrad, measured_omega_mradps, max_rate_mradps);
    Wulong_SetBodyVelocity(0.0f, omega_cmd);
    if ((abs_error < error_tol_mrad) &&
        (fabsf(measured_omega_mradps) < 30.0f)) {
      Wulong_Stop();
      k3_head_cycle_phase = 1U;
      k3_head_phase_ms = now_ms;
    }
    break;

  case 1U:  /* coast without torque */
    Wulong_Stop();
    if (phase_elapsed_ms >= 220U) {
      k3_head_cycle_phase = 2U;
      k3_head_phase_ms = now_ms;
    }
    break;

  default:  /* stationary verification */
    Wulong_Stop();
    if (phase_elapsed_ms < 350U) {
      break;
    }

    ++k3_head_attempts;
    if ((fabsf(error_mrad) <= error_tol_mrad) &&
        (fabsf(measured_omega_mradps) <= rate_tol_mradps)) {
      return 1U;
    }
    if (k3_head_attempts >= max_attempts) {
      return 2U;
    }

    k3_head_cycle_phase = 0U;
    k3_head_phase_ms = now_ms;
    break;
  }

  return 0U;
}

static float MotorTest_K3NextOmega(void)
{
  uint32_t r;
  float magnitude;

  /* xorshift32: deterministic, cheap, and re-seeded by the live tick/gyro. */
  r = k3_random_state;
  r ^= r << 13;
  r ^= r >> 17;
  r ^= r << 5;
  k3_random_state = r;

  /*
   * Keep the wheel differential well below the 30 % duty ceiling.  The old
   * 2200-3000 mrad/s command saturated the wheels and used roller scrub as the
   * dominant "motion", which made the measured position drift much worse.
   */
  magnitude = 700.0f + (float)(r % 300U);
  return ((r & 0x80000000UL) != 0U) ? magnitude : -magnitude;
}

static void MotorTest_K3Finish(const char *result)
{
  Wulong_Pose pose = Wulong_GetPose();
  char buf[128];

  Wulong_Stop();
  Wulong_ControlDisarm();
  MotorTest_Disarm();
  k3_test_active = 0U;
  k3_last_result = result;
  k3_state = (strcmp(result, "OK") == 0) ? K3_TEST_DONE : K3_TEST_ERROR;

  (void)snprintf(buf, sizeof(buf),
                 "K3 DONE result=%s x=%.1f y=%.1f yaw=%.1f pos_err=%.1f hdg_err=%.1f\r\n",
                 result, (double)pose.x_mm, (double)pose.y_mm,
                 (double)pose.theta_mrad, (double)k3_position_error_mm,
                 (double)k3_heading_error_mrad);
  Uart_SendText(buf);
}

static void MotorTest_K3Start(void)
{
  uint32_t now;

  if (k3_test_active) {
    Uart_SendText("K3 BUSY\r\n");
    return;
  }
  if (!Imu_IsReady()) {
    Uart_SendText("K3 ERR: IMU not ready\r\n");
    return;
  }
  if (speed_tune_active) {
    MotorTest_SpeedTuneFinish("PREEMPTED");
  }

  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  all_full_active = 0U;
  g_ctrl_console_active = 0U;
  g_imu_telemetry = 0U;
  g_imu_report = 0U;
  MotorTest_StopOutputs();
  MotorTest_Disarm();

  g_imu_auto_cal_suppressed = 1U;
  Wulong_ControlInit();
  Wulong_ControlDisarm();
  MotorTest_Disarm();
  Imu_StartBiasCalibration(2000U);
  Wulong_ResetPose(0.0f, 0.0f, 0.0f);

  now = HAL_GetTick();
  k3_random_state ^= now ^ ((uint32_t)(uint16_t)Imu_GetGyroZRaw() << 16);
  if (k3_random_state == 0U) {
    k3_random_state = 0x6D2B79F5UL;
  }
  k3_state = K3_TEST_CALIBRATING;
  k3_state_ms = now;
  k3_target_omega_mradps = 0.0f;
  k3_spin_cmd_mradps = 0.0f;
  k3_spin_rate_integral = 0.0f;
  k3_position_error_mm = 0.0f;
  k3_heading_error_mrad = 0.0f;
  k3_cal_seen = 0U;
  k3_heading_retry = 0U;
  k3_heading_recoveries = 0U;
  k3_heading_best_abs_mrad = 0.0f;
  k3_heading_progress_ms = now;
  k3_position_corrections = 0U;
  k3_position_best_mm = 0.0f;
  k3_position_progress_ms = now;
  MotorTest_K3HeadingCycleReset();
  k3_last_result = "running";
  k3_gyro_trace_count = 0U;
  k3_gyro_trace_last_ms = now - 20U;
  k3_test_active = 1U;

  Uart_SendText("K3 START: calibrate 2000 samples, keep chassis still\r\n");
}

static void MotorTest_K3TraceSample(uint32_t now_ms)
{
  K3_GyroTraceEntry *entry;
  Wulong_Pose pose;
  float command_vx;
  float command_vy;
  float command_omega;

  if (k3_gyro_trace_count >= K3_TRACE_CAPACITY) {
    return;
  }
  if ((uint32_t)(now_ms - k3_gyro_trace_last_ms) < 20U) {
    return;
  }

  k3_gyro_trace_last_ms = now_ms;
  pose = Wulong_GetPose();
  entry = &k3_gyro_trace[k3_gyro_trace_count++];
  entry->t_ms = now_ms;
  entry->yaw_mrad = Imu_GetYawMrad();
  entry->gx_dps = Imu_GetGyroXDps();
  entry->gy_dps = Imu_GetGyroYDps();
  entry->gz_dps = Imu_GetGyroZDps();
  entry->bias_x_dps = Imu_GetGyroXBiasDps();
  entry->bias_y_dps = Imu_GetGyroYBiasDps();
  entry->bias_z_dps = Imu_GetGyroZBiasDps();
  entry->heading_error_mrad = MotorTest_WrapMrad(-entry->yaw_mrad);
  entry->position_error_mm =
      sqrtf((pose.x_mm * pose.x_mm) + (pose.y_mm * pose.y_mm));
  entry->x_mm = pose.x_mm;
  entry->y_mm = pose.y_mm;
  Wulong_GetBodyVelocity3(&command_vx, &command_vy, &command_omega);
  entry->command_vx_mmps = command_vx;
  entry->command_vy_mmps = command_vy;
  entry->command_omega_mradps = command_omega;
  entry->target_omega_mradps = (int16_t)k3_target_omega_mradps;
  entry->state = (uint8_t)k3_state;
  entry->stationary = Imu_IsStationary();
  entry->active = k3_test_active;
  entry->reserved = 0U;
}

static void MotorTest_K3Step(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  Wulong_Pose pose;
  float measured_omega;
  float error_mrad;
  float omega_cmd;
  float distance_mm;
  uint8_t heading_result;

  if (!k3_test_active) {
    return;
  }

  now = HAL_GetTick();
  elapsed_ms = (uint32_t)(now - k3_state_ms);
  MotorTest_K3TraceSample(now);

  switch (k3_state) {
  case K3_TEST_CALIBRATING:
    if (Imu_IsCalibrating()) {
      k3_cal_seen = 1U;
    }
    if (k3_cal_seen && Imu_CalibrationDone()) {
      Wulong_ResetPose(0.0f, 0.0f, 0.0f);
      Wulong_ControlArm();
      k3_target_omega_mradps = MotorTest_K3NextOmega();
      k3_spin_cmd_mradps = 0.0f;
      k3_spin_rate_integral = 0.0f;
      k3_state = K3_TEST_SPIN;
      k3_state_ms = now;
      Wulong_SetBodyVelocityXY(0.0f, 0.0f, 0.0f);
      Uart_SendText("K3 GYRO CALIBRATED\r\n");
      Uart_SendText("K3 SPIN target_mradps=");
      Uart_SendS32((int32_t)k3_target_omega_mradps);
      Uart_SendText(" duration_ms=5000\r\n");
    } else if (elapsed_ms >= 6000U) {
      MotorTest_K3Finish("CAL_TIMEOUT");
    }
    break;

  case K3_TEST_SPIN:
    {
      float target_rate = k3_target_omega_mradps;
      float measured_rate = Imu_GetGyroZDps() * 17.4532925f;
      float rate_error;

      if (elapsed_ms < K3_SPIN_RAMP_MS) {
        target_rate *=
            ((float)elapsed_ms / (float)K3_SPIN_RAMP_MS);
      }

      rate_error = target_rate - measured_rate;
      k3_spin_rate_integral += rate_error * 0.001f;
      k3_spin_rate_integral =
          MotorTest_ClampFloat(k3_spin_rate_integral,
                               -K3_SPIN_RATE_INTEGRAL_LIMIT,
                               K3_SPIN_RATE_INTEGRAL_LIMIT);
      k3_spin_cmd_mradps =
          (K3_SPIN_RATE_FF * target_rate) +
          (K3_SPIN_RATE_KP * rate_error) +
          (K3_SPIN_RATE_KI * k3_spin_rate_integral);
      k3_spin_cmd_mradps =
          MotorTest_ClampFloat(k3_spin_cmd_mradps,
                               -K3_SPIN_COMMAND_LIMIT,
                               K3_SPIN_COMMAND_LIMIT);
    }
    Wulong_SetBodyVelocityXY(0.0f, 0.0f, k3_spin_cmd_mradps);
    if (elapsed_ms >= 5000U) {
      k3_spin_cmd_mradps = 0.0f;
      k3_spin_rate_integral = 0.0f;
      Wulong_SetBodyVelocityXY(0.0f, 0.0f, 0.0f);
      k3_state = K3_TEST_BRAKE;
      k3_state_ms = now;
    }
    break;

  case K3_TEST_BRAKE:
    /*
     * Keep the speed loop armed with zero targets so it actively opposes the
     * remaining wheel motion.  Wulong_Stop() would free-run and let the chassis
     * scrub farther away from the calibrated point.
     */
    Wulong_SetBodyVelocityXY(0.0f, 0.0f, 0.0f);
    if (elapsed_ms >= K3_ACTIVE_BRAKE_MS) {
      Wulong_Stop();
      k3_state = K3_TEST_HEADING_RETURN;
      k3_state_ms = now;
      MotorTest_K3HeadingCycleReset();
      k3_heading_best_abs_mrad = 1.0e9f;
      k3_heading_progress_ms = now;
    }
    break;

  case K3_TEST_HEADING_RETURN:
    pose = Wulong_GetPose();
    measured_omega = Imu_GetGyroZDps() * 17.4532925f;
    error_mrad = MotorTest_WrapMrad(-pose.theta_mrad);
    k3_heading_error_mrad = error_mrad;

    if (fabsf(error_mrad) + 25.0f < k3_heading_best_abs_mrad) {
      k3_heading_best_abs_mrad = fabsf(error_mrad);
      k3_heading_progress_ms = now;
    }

    /*
     * A 120-180 degree start can otherwise pass through the wrap boundary
     * repeatedly while the wheels chase a saturated rate command.  If the
     * heading makes no meaningful progress, remove torque and restart after
     * the normal brake interval with fresh rate-filter state.
     */
    if ((elapsed_ms >= K3_HEADING_STALL_MS) &&
        ((uint32_t)(now - k3_heading_progress_ms) >= K3_HEADING_STALL_MS) &&
        (k3_heading_best_abs_mrad > 80.0f)) {
      if (k3_heading_recoveries < K3_HEADING_MAX_RECOVERIES) {
        ++k3_heading_recoveries;
        Wulong_Stop();
        k3_state = K3_TEST_BRAKE;
        k3_state_ms = now;
        Uart_SendText("K3 HEADING STALL RECOVERY\r\n");
      } else {
        MotorTest_K3Finish("HEADING_STALL");
      }
      break;
    }

    heading_result = MotorTest_K3HeadingCycle(
        error_mrad, measured_omega, now, 1400.0f, 10U, 8.0f, 8.0f);
    if (heading_result == 1U) {
      Wulong_Stop();
      k3_state = K3_TEST_POSITION_CHECK;
      k3_state_ms = now;
      break;
    }
    if (heading_result == 2U) {
      MotorTest_K3Finish("HEADING_VERIFY_FAIL");
      break;
    }
    if (elapsed_ms >= 20000U) {
      MotorTest_K3Finish("HEADING_TIMEOUT");
      break;
    }
    break;

  case K3_TEST_POSITION_CHECK:
    pose = Wulong_GetPose();
    distance_mm = sqrtf(pose.x_mm * pose.x_mm + pose.y_mm * pose.y_mm);
    k3_position_error_mm = distance_mm;
    k3_heading_error_mrad = MotorTest_WrapMrad(-pose.theta_mrad);

    if ((distance_mm <= K3_POSITION_ACCEPT_MM) &&
        (fabsf(k3_heading_error_mrad) <= 8.0f)) {
      MotorTest_K3Finish("OK");
    } else if (distance_mm <= K3_POSITION_ACCEPT_MM) {
      /*
       * Position is already good, but the heading verification failed.  Repeat
       * the heading loop without mixing in a translation command.
       */
      if (k3_heading_retry < 2U) {
        ++k3_heading_retry;
        k3_state = K3_TEST_HEADING_RETURN;
        k3_state_ms = now;
        MotorTest_K3HeadingCycleReset();
        Uart_SendText("K3 HEADING RECHECK\r\n");
      } else {
        MotorTest_K3Finish("HEADING_RECHECK_FAIL");
      }
    } else if (distance_mm >= K3_POSITION_LIMIT_MM) {
      MotorTest_K3Finish("POSITION_LIMIT");
    } else if (k3_position_corrections >= K3_POSITION_MAX_CORRECTIONS) {
      MotorTest_K3Finish("POSITION_VERIFY_FAIL");
    } else {
      ++k3_position_corrections;
      k3_state = K3_TEST_POSITION_DRIVE;
      k3_state_ms = now;
      k3_heading_retry = 0U;
      k3_position_best_mm = distance_mm;
      k3_position_progress_ms = now;
      MotorTest_K3HeadingCycleReset();
      Uart_SendText("K3 MECANUM POSITION RETURN\r\n");
    }
    break;

  case K3_TEST_POSITION_DRIVE:
    pose = Wulong_GetPose();
    distance_mm = sqrtf(pose.x_mm * pose.x_mm + pose.y_mm * pose.y_mm);
    k3_position_error_mm = distance_mm;
    k3_heading_error_mrad = MotorTest_WrapMrad(-pose.theta_mrad);

    if (distance_mm + 2.0f < k3_position_best_mm) {
      k3_position_best_mm = distance_mm;
      k3_position_progress_ms = now;
    }

    if (distance_mm <= K3_POSITION_STOP_MM) {
      Wulong_Stop();
      k3_state = K3_TEST_FINAL_HEADING;
      k3_state_ms = now;
      k3_heading_retry = 0U;
      MotorTest_K3HeadingCycleReset();
      k3_heading_best_abs_mrad = 1.0e9f;
      k3_heading_progress_ms = now;
      break;
    }
    if (elapsed_ms >= K3_POSITION_TIMEOUT_MS) {
      MotorTest_K3Finish("POSITION_TIMEOUT");
      break;
    }
    if ((elapsed_ms >= K3_POSITION_STALL_MS) &&
        ((uint32_t)(now - k3_position_progress_ms) >=
         K3_POSITION_STALL_MS)) {
      MotorTest_K3Finish("POSITION_STALL");
      break;
    }

    measured_omega = Imu_GetGyroZDps() * 17.4532925f;
    error_mrad = MotorTest_WrapMrad(-pose.theta_mrad);
    omega_cmd = MotorTest_K3HeadingCommand(error_mrad, measured_omega, 350.0f);
    MotorTest_K3ApplyPositionReturn(&pose, omega_cmd);
    break;

  case K3_TEST_FINAL_HEADING:
    pose = Wulong_GetPose();
    measured_omega = Imu_GetGyroZDps() * 17.4532925f;
    error_mrad = MotorTest_WrapMrad(-pose.theta_mrad);
    k3_heading_error_mrad = error_mrad;

    heading_result = MotorTest_K3HeadingCycle(
        error_mrad, measured_omega, now, 900.0f, 10U, 8.0f, 8.0f);
    if (heading_result == 1U) {
      Wulong_Stop();
      k3_state = K3_TEST_POSITION_CHECK;
      k3_state_ms = now;
      break;
    }
    if (heading_result == 2U) {
      MotorTest_K3Finish("FINAL_HEADING_VERIFY_FAIL");
      break;
    }
    if (elapsed_ms >= 20000U) {
      MotorTest_K3Finish("FINAL_HEADING_TIMEOUT");
      break;
    }
    break;

  default:
    break;
  }
}

static void MotorTest_SpeedTuneFinish(const char *result)
{
  char buf[160];
  uint8_t phase;
  uint8_t wheel;

  Wulong_Stop();
  Wulong_ControlDisarm();
  MotorTest_Disarm();
  speed_tune_active = 0U;
  speed_tune_done = (uint8_t)((strcmp(result, "OK") == 0) ? 1U : 2U);
  Uart_SendText("STUNE DONE result=");
  Uart_SendText(result);
  Uart_SendText("\r\n");
  for (phase = 0U; phase < SPEED_TUNE_PHASE_COUNT; ++phase) {
    for (wheel = 0U; wheel < WULONG_WHEEL_COUNT; ++wheel) {
      (void)snprintf(
          buf, sizeof(buf),
          "STUNE p=%u w=%u mean=%.1f ripple=%.1f std=%.1f duty=%.1f rise=%.0fms\r\n",
          (unsigned int)phase, (unsigned int)wheel,
          (double)speed_tune_mean[phase][wheel],
          (double)speed_tune_ripple[phase][wheel],
          (double)speed_tune_std[phase][wheel],
          (double)speed_tune_duty[phase][wheel],
          (double)speed_tune_rise_ms[phase][wheel]);
      Uart_SendText(buf);
    }
  }
}

static void MotorTest_SpeedTuneStart(void)
{
  uint8_t i;
  uint8_t phase;

  if (speed_tune_active || k3_test_active) {
    Uart_SendText("STUNE BUSY\r\n");
    return;
  }

  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  all_full_active = 0U;
  g_ctrl_console_active = 0U;
  g_imu_telemetry = 0U;
  g_imu_report = 0U;
  MotorTest_StopOutputs();
  MotorTest_Disarm();

  for (phase = 0U; phase < SPEED_TUNE_PHASE_COUNT; ++phase) {
    for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
      speed_tune_rise_ms[phase][i] = -1.0f;
      speed_tune_mean[phase][i] = 0.0f;
      speed_tune_ripple[phase][i] = 0.0f;
      speed_tune_std[phase][i] = 0.0f;
      speed_tune_duty[phase][i] = 0.0f;
    }
  }

  g_imu_auto_cal_suppressed = 1U;
  Wulong_ControlInit();
  Wulong_ControlArm();
  speed_tune_phase = 0U;
  speed_tune_phase_ms = HAL_GetTick();
  speed_tune_done = 0U;
  speed_tune_active = 1U;
  speed_tune_sample_count = 0U;
  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    speed_tune_sum[i] = 0.0f;
    speed_tune_sum_sq[i] = 0.0f;
    speed_tune_duty_sum[i] = 0.0f;
    speed_tune_min[i] = 1.0e6f;
    speed_tune_max[i] = -1.0e6f;
  }
  Wulong_SetBodyVelocity(speed_tune_targets[0], 0.0f);

  Uart_SendText("STUNE START targets=100 150 200 mm/s\r\n");
}

static void MotorTest_SpeedTuneStep(void)
{
  uint32_t now;
  uint32_t elapsed_ms;
  uint8_t i;
  float target_mmps;
  float speed_mmps;

  if (!speed_tune_active) {
    return;
  }
  if (speed_tune_phase >= SPEED_TUNE_PHASE_COUNT) {
    MotorTest_SpeedTuneFinish("OK");
    return;
  }

  now = HAL_GetTick();
  elapsed_ms = (uint32_t)(now - speed_tune_phase_ms);
  target_mmps = speed_tune_targets[speed_tune_phase];
  Wulong_SetBodyVelocity(target_mmps, 0.0f);

  if (elapsed_ms >= SPEED_TUNE_PHASE_MS) {
    if (speed_tune_sample_count == 0U) {
      MotorTest_SpeedTuneFinish("NO_SAMPLES");
      return;
    }
    for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
      float avg = speed_tune_sum[i] / (float)speed_tune_sample_count;
      float mean_sq = speed_tune_sum_sq[i] / (float)speed_tune_sample_count;
      float duty = speed_tune_duty_sum[i] / (float)speed_tune_sample_count;
      float variance = mean_sq - avg * avg;

      if (variance < 0.0f) {
        variance = 0.0f;
      }
      speed_tune_mean[speed_tune_phase][i] = avg;
      speed_tune_ripple[speed_tune_phase][i] =
          speed_tune_max[i] - speed_tune_min[i];
      speed_tune_std[speed_tune_phase][i] = sqrtf(variance);
      speed_tune_duty[speed_tune_phase][i] = duty;
      speed_tune_sum[i] = 0.0f;
      speed_tune_sum_sq[i] = 0.0f;
      speed_tune_duty_sum[i] = 0.0f;
      speed_tune_min[i] = 1.0e6f;
      speed_tune_max[i] = -1.0e6f;
    }
    speed_tune_sample_count = 0U;

    ++speed_tune_phase;
    if (speed_tune_phase >= SPEED_TUNE_PHASE_COUNT) {
      MotorTest_SpeedTuneFinish("OK");
      return;
    }
    speed_tune_phase_ms = now;
    Wulong_SetBodyVelocity(speed_tune_targets[speed_tune_phase], 0.0f);
    return;
  }

  if (elapsed_ms < SPEED_TUNE_SETTLE_MS) {
    for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
      speed_mmps = Wulong_GetWheelSpeed((Wulong_Wheel)i);
      if ((speed_tune_rise_ms[speed_tune_phase][i] < 0.0f) &&
          (speed_mmps >= target_mmps * 0.90f)) {
        speed_tune_rise_ms[speed_tune_phase][i] = (float)elapsed_ms;
      }
    }
    return;
  }

  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    speed_mmps = Wulong_GetWheelSpeed((Wulong_Wheel)i);
    speed_tune_sum[i] += speed_mmps;
    speed_tune_sum_sq[i] += speed_mmps * speed_mmps;
    speed_tune_duty_sum[i] +=
        Wulong_GetWheelDuty((Wulong_Wheel)i);
    if (speed_mmps < speed_tune_min[i]) {
      speed_tune_min[i] = speed_mmps;
    }
    if (speed_mmps > speed_tune_max[i]) {
      speed_tune_max[i] = speed_mmps;
    }
    if ((speed_tune_rise_ms[speed_tune_phase][i] < 0.0f) &&
        (speed_mmps >= target_mmps * 0.90f)) {
      speed_tune_rise_ms[speed_tune_phase][i] = (float)elapsed_ms;
    }
  }
  ++speed_tune_sample_count;
}


static void MotorTest_K1SeqStep(void)
{
  uint32_t elapsed_ms;

  if (!k1_seq_active) {
    return;
  }

  elapsed_ms = (uint32_t)(HAL_GetTick() - k1_seq_phase_start_ms);
  if (elapsed_ms >= K1_SEQ_PHASE_MS) {
    MotorTest_StopOutputs();
    if (++k1_seq_phase >= K1_SEQ_PHASE_COUNT) {
      k1_seq_active = 0U;
      MotorTest_Disarm();
      Uart_SendText("K1 SEQ DONE\r\n");
      return;
    }

    MotorTest_Arm();
    k1_seq_phase_start_ms = HAL_GetTick();
    Uart_SendText("K1 PHASE ");
    Uart_SendText(k1_seq_phase_names[k1_seq_phase]);
    Uart_SendText("\r\n");
  }

  MotorTest_K1SeqApply(k1_seq_phase);
}

static void MotorTest_KeyScan(void)
{
  static uint8_t  k1_stable = 1U;
  static uint8_t  k1_last_raw = 1U;
  static uint32_t k1_last_change_ms = 0U;
  static uint8_t  k2_stable = 1U;
  static uint8_t  k2_last_raw = 1U;
  static uint32_t k2_last_change_ms = 0U;
  static uint8_t  k3_stable = 1U;
  static uint8_t  k3_last_raw = 1U;
  static uint32_t k3_last_change_ms = 0U;
  uint8_t raw_k1;
  uint8_t raw_k2;
  uint8_t raw_k3;
  uint32_t now;

  now = HAL_GetTick();
  raw_k1 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET) ? 0U : 1U;
  raw_k2 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == GPIO_PIN_RESET) ? 0U : 1U;
  raw_k3 = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_RESET) ? 0U : 1U;

  if (raw_k1 != k1_last_raw) {
    k1_last_raw = raw_k1;
    k1_last_change_ms = now;
  } else if ((uint32_t)(now - k1_last_change_ms) >= 30U &&
             raw_k1 != k1_stable) {
    k1_stable = raw_k1;
    if (k1_stable == 0U) {
      if (!g_vision_approach_active && !g_ctrl_console_active &&
          !k3_test_active && !speed_tune_active &&
          !MotorTest_AutoIsActive() &&
          !k1_seq_active && !k2_ramp_active && !turn_right_active &&
          !drive_mode_active && !all_full_active) {
        Uart_SendText("K1 BUTTON: vision approach\r\n");
        VisionApproachStart();
      } else {
        Uart_SendText("K1 ignored: system busy\r\n");
      }
    }
  }

  if (raw_k2 != k2_last_raw) {
    k2_last_raw = raw_k2;
    k2_last_change_ms = now;
  } else if ((uint32_t)(now - k2_last_change_ms) >= 30U &&
             raw_k2 != k2_stable) {
    k2_stable = raw_k2;
    if (k2_stable == 0U) {
      MotorTest_K2RampStart();
    }
  }

  if (raw_k3 != k3_last_raw) {
    k3_last_raw = raw_k3;
    k3_last_change_ms = now;
  } else if ((uint32_t)(now - k3_last_change_ms) >= 30U &&
             raw_k3 != k3_stable) {
    k3_stable = raw_k3;
    if (k3_stable == 0U) {
      MotorTest_K3Start();
    }
  }
}

static void MotorTest_Start(uint8_t motor, int8_t dir, uint16_t permille, uint32_t duration_ms)
{
  if (motor > 3U || dir == 0U) {
    return;
  }
  if (permille == 0U) {
    permille = 1U;
  }
  if (permille > 300U) {
    permille = 300U;
  }
  if (duration_ms == 0U) {
    duration_ms = 1U;
  }
  if (duration_ms > 5000U) {
    duration_ms = 5000U;
  }

  active_motor = motor;
  active_dir = dir;
  active_until_ms = HAL_GetTick() + duration_ms;
  MotorTest_SetOutput(motor, dir, permille, 0U);
}

static void MotorTest_RunSingle(uint8_t motor, int8_t dir)
{
  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();
  MotorTest_Start(motor, dir, 100U, 800U);

  Uart_SendText("TEST ");
  Uart_SendMotor(motor);
  Uart_SendText(dir > 0 ? "+" : "-");
  Uart_SendText(" duty=100 ms=800\r\n");
}

/*
 * AUTO TEST order:
 *   A+ A- B+ B- C+ C- D+ D-
 * Each run is 800 ms at 10%, followed by a 300 ms pause.  The encoder delta
 * is printed after every run so wiring, direction and feedback can be checked
 * without issuing additional commands.
 */
static volatile uint8_t  auto_test_active = 0;
static uint8_t           auto_test_index = 0;
static uint8_t           auto_test_phase = 0;   /* 0=load, 1=run, 2=gap */
static uint32_t          auto_test_deadline_ms = 0;
static int32_t           auto_test_start_count = 0;

static uint8_t MotorTest_AutoIsActive(void)
{
  return auto_test_active;
}

static void MotorTest_AutoStart(void)
{
  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  auto_test_active = 1U;
  auto_test_index = 0U;
  auto_test_phase = 0U;
  auto_test_deadline_ms = HAL_GetTick();
  Uart_SendText("AUTO TEST START A+ A- B+ B- C+ C- D+ D-\r\n");
}

static void MotorTest_AutoCancel(void)
{
  if (auto_test_active) {
    auto_test_active = 0U;
    auto_test_phase = 0U;
    MotorTest_StopOutputs();
    Uart_SendText("AUTO TEST CANCEL\r\n");
  }
}

static void MotorTest_AutoStep(void)
{
  uint8_t motor;
  int8_t dir;
  uint32_t now;

  if (!auto_test_active) {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - auto_test_deadline_ms) < 0) {
    return;
  }

  if (auto_test_phase == 0U) {
    if (auto_test_index >= 8U) {
      auto_test_active = 0U;
      auto_test_phase = 0U;
      MotorTest_StopOutputs();
      Uart_SendText("AUTO TEST DONE\r\n");
      return;
    }

    motor = (uint8_t)(auto_test_index / 2U);
    dir = ((auto_test_index % 2U) == 0U) ? 1 : -1;
    auto_test_start_count = MotorTest_ReadEncoder(motor);
    MotorTest_SetOutput(motor, dir, 100U, 0U);
    auto_test_phase = 1U;
    auto_test_deadline_ms = now + 800U;

    Uart_SendText("AUTO ");
    Uart_SendMotor(motor);
    Uart_SendText(dir > 0 ? "+" : "-");
    Uart_SendText("\r\n");
  } else if (auto_test_phase == 1U) {
    motor = (uint8_t)(auto_test_index / 2U);
    dir = ((auto_test_index % 2U) == 0U) ? 1 : -1;

    MotorTest_StopOutputs();
    Uart_SendText("DONE ");
    Uart_SendMotor(motor);
    Uart_SendText(dir > 0 ? "+" : "-");
    Uart_SendText(" delta=");
    Uart_SendS32(MotorTest_ReadEncoder(motor) - auto_test_start_count);
    Uart_SendText("\r\n");

    auto_test_phase = 2U;
    auto_test_deadline_ms = now + 300U;
  } else {
    ++auto_test_index;
    MotorTest_Arm();
    auto_test_phase = 0U;
    auto_test_deadline_ms = now;
  }
}

static int32_t MotorTest_ReadEncoder(uint8_t motor)
{
  int32_t value;

  switch (motor) {
  case 0:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    break;
  case 1:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    break;
  case 2:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    break;
  case 3:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim20);
    break;
  default:
    return 0;
  }

  /* Flipped wheels also have their direction lines inverted - see FLIP. */
  return Wulong_GetWheelFlip((Wulong_Wheel)motor) ? -value : value;
}

static void MotorTest_ResetEncoders(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim20, 0);
}

static uint32_t MotorTest_ReadBatteryMv(void)
{
  uint32_t raw = 0;

  if (HAL_ADC_Start(&hadc2) != HAL_OK) {
    return 0;
  }
  if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
    raw = HAL_ADC_GetValue(&hadc2);
  }
  HAL_ADC_Stop(&hadc2);

  /* D24A ADC output is 1/11 of battery voltage; ADC reference is 3.3 V. */
  return (uint32_t)(((uint64_t)raw * 3300ULL * 11ULL) / 4095ULL);
}

static const char *SkipSpaces(const char *p)
{
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  return p;
}

static uint8_t ParseU32(const char **p, uint32_t *value)
{
  const char *s = SkipSpaces(*p);
  uint32_t v = 0;
  uint8_t got = 0;

  while (*s >= '0' && *s <= '9') {
    v = (v * 10U) + (uint32_t)(*s - '0');
    ++s;
    got = 1;
  }
  *p = s;
  *value = v;
  return got;
}

/*
 * Minimal float parser: optional sign, integer part, optional fraction.
 * No exponent support (not needed for calibration values).
 */
static uint8_t ParseFloat(const char **p, float *value)
{
  const char *s = SkipSpaces(*p);
  float sign = 1.0f;
  float v = 0.0f;
  float frac = 0.1f;
  uint8_t got = 0;

  if (*s == '-') {
    sign = -1.0f;
    ++s;
  } else if (*s == '+') {
    ++s;
  }

  while (*s >= '0' && *s <= '9') {
    v = (v * 10.0f) + (float)(*s - '0');
    ++s;
    got = 1;
  }

  if (*s == '.') {
    ++s;
    while (*s >= '0' && *s <= '9') {
      v += (float)(*s - '0') * frac;
      frac *= 0.1f;
      ++s;
      got = 1;
    }
  }

  *p = s;
  *value = sign * v;
  return got;
}

static uint8_t ParseMotor(const char **p, uint8_t *motor)
{
  const char *s = SkipSpaces(*p);

  if (*s < 'A' || *s > 'D') {
    return 0;
  }
  *motor = (uint8_t)(*s - 'A');
  *p = s + 1;
  return 1;
}

static void TrimCommand(char *line)
{
  char *start = line;
  size_t len;

  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
    ++start;
  }
  if (start != line) {
    memmove(line, start, strlen(start) + 1U);
  }

  len = strlen(line);
  while (len > 0U &&
         (line[len - 1U] == ' ' || line[len - 1U] == '\t' ||
          line[len - 1U] == '\r' || line[len - 1U] == '\n' ||
          line[len - 1U] == ';')) {
    line[--len] = '\0';
  }
}

static void Uart_SendText(const char *text)
{
  if (text != NULL) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
  }
}

/*
 * USART1 RX is interrupt-driven.  Each completion re-arms reception of one
 * byte and pushes it into the ring buffer for CommsTask.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint16_t next = (uint16_t)((s_cli_rx_head + 1U) % CLI_RX_BUF_SIZE);

    if (next != s_cli_rx_tail) {
      s_cli_rx_buf[s_cli_rx_head] = s_cli_rx_byte;
      s_cli_rx_head = next;
    }
    ++s_cli_rx_count;
    (void)HAL_UART_Receive_IT(&huart1, &s_cli_rx_byte, 1U);
  } else if (huart->Instance == USART2) {
    uint16_t next = (uint16_t)((s_k230_rx_head + 1U) % K230_RX_BUF_SIZE);

    if (next != s_k230_rx_tail) {
      s_k230_rx_buf[s_k230_rx_head] = s_k230_rx_byte;
      s_k230_rx_head = next;
    }
    ++s_k230_rx_count;
    (void)HAL_UART_Receive_IT(&huart2, &s_k230_rx_byte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    /* Clear the error and re-arm; otherwise an overrun leaves RX dead for
     * good and the console stops answering entirely. */
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart1, &s_cli_rx_byte, 1U);
  } else if (huart->Instance == USART2) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart2, &s_k230_rx_byte, 1U);
  }
}

static void CliRxStart(void)
{
  s_cli_rx_head = 0U;
  s_cli_rx_tail = 0U;
  s_cli_rx_count = 0U;
  (void)HAL_UART_Receive_IT(&huart1, &s_cli_rx_byte, 1U);
}

static uint8_t CliGetByte(uint8_t *byte)
{
  if (s_cli_rx_tail == s_cli_rx_head) {
    return 0U;
  }
  *byte = s_cli_rx_buf[s_cli_rx_tail];
  s_cli_rx_tail = (uint16_t)((s_cli_rx_tail + 1U) % CLI_RX_BUF_SIZE);
  return 1U;
}

static void K230RxStart(void)
{
  s_k230_rx_head = 0U;
  s_k230_rx_tail = 0U;
  s_k230_rx_count = 0U;
  s_k230_frame_len = 0U;
  s_vision_err_pm = 0;
  s_vision_size_pm = 0;
  s_vision_conf = 0U;
  s_vision_valid = 0U;
  s_vision_last_ms = 0U;
  s_vision_frame_count = 0U;
  s_vision_crc_errors = 0U;
  (void)HAL_UART_Receive_IT(&huart2, &s_k230_rx_byte, 1U);
}

static uint8_t K230GetByte(uint8_t *byte)
{
  if (s_k230_rx_tail == s_k230_rx_head) {
    return 0U;
  }
  *byte = s_k230_rx_buf[s_k230_rx_tail];
  s_k230_rx_tail = (uint16_t)((s_k230_rx_tail + 1U) % K230_RX_BUF_SIZE);
  return 1U;
}

static uint16_t K230Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static void K230HandlePayload(const uint8_t *payload, uint16_t length)
{
  uint8_t msg_id;

  if (length == 0U) {
    return;
  }
  msg_id = payload[0];

  if (msg_id == K230_MSG_APPROACH_TARGET && length == 7U) {
    int16_t err_pm = (int16_t)((uint16_t)payload[1] |
                               ((uint16_t)payload[2] << 8));
    int16_t size_pm = (int16_t)((uint16_t)payload[3] |
                                ((uint16_t)payload[4] << 8));

    err_pm = (err_pm > 1000) ? 1000 : err_pm;
    err_pm = (err_pm < -1000) ? -1000 : err_pm;
    size_pm = (size_pm > 1000) ? 1000 : size_pm;
    size_pm = (size_pm < 0) ? 0 : size_pm;
    s_vision_err_pm = err_pm;
    s_vision_size_pm = size_pm;
    s_vision_conf = (payload[5] > 100U) ? 100U : payload[5];
    s_vision_valid = (uint8_t)(payload[6] ? 1U : 0U);
    s_vision_last_ms = HAL_GetTick();
    ++s_vision_frame_count;
  } else if (msg_id == K230_MSG_HELLO) {
    /* HELLO is informational for now.  The control path remains driven by
     * APPROACH_TARGET and its timeout. */
  }
}

static void K230ParserFeed(uint8_t byte)
{
  uint16_t payload_length;
  uint16_t frame_length;
  uint16_t crc_rx;
  uint16_t crc_calc;

  if (s_k230_frame_len >= K230_FRAME_BUF_SIZE) {
    memmove(s_k230_frame_buf, &s_k230_frame_buf[1],
            K230_FRAME_BUF_SIZE - 1U);
    s_k230_frame_len = K230_FRAME_BUF_SIZE - 1U;
  }
  s_k230_frame_buf[s_k230_frame_len++] = byte;

  for (;;) {
    if (s_k230_frame_len < 2U) {
      return;
    }
    if (s_k230_frame_buf[0] != 0xAAU ||
        s_k230_frame_buf[1] != 0x55U) {
      memmove(s_k230_frame_buf, &s_k230_frame_buf[1],
              s_k230_frame_len - 1U);
      --s_k230_frame_len;
      continue;
    }
    if (s_k230_frame_len < 3U) {
      return;
    }

    payload_length = s_k230_frame_buf[2];
    if (payload_length > K230_MAX_PAYLOAD) {
      memmove(s_k230_frame_buf, &s_k230_frame_buf[1],
              s_k230_frame_len - 1U);
      --s_k230_frame_len;
      continue;
    }

    frame_length = (uint16_t)(6U + payload_length);
    if (s_k230_frame_len < frame_length) {
      return;
    }

    crc_rx = (uint16_t)s_k230_frame_buf[3U + payload_length] |
             ((uint16_t)s_k230_frame_buf[4U + payload_length] << 8);
    crc_calc = K230Crc16(&s_k230_frame_buf[2],
                         (uint16_t)(payload_length + 1U));
    if (crc_rx != crc_calc || s_k230_frame_buf[5U + payload_length] != 0x0DU) {
      ++s_vision_crc_errors;
      memmove(s_k230_frame_buf, &s_k230_frame_buf[1],
              s_k230_frame_len - 1U);
      --s_k230_frame_len;
      continue;
    }

    K230HandlePayload(&s_k230_frame_buf[3], payload_length);
    memmove(s_k230_frame_buf, &s_k230_frame_buf[frame_length],
            s_k230_frame_len - frame_length);
    s_k230_frame_len = (uint16_t)(s_k230_frame_len - frame_length);
  }
}

static void VisionApproachStart(void)
{
  MotorTest_AutoCancel();
  k1_seq_active = 0U;
  k2_ramp_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  all_full_active = 0U;
  g_ctrl_console_active = 0U;
  g_imu_telemetry = 0U;
  g_imu_report = 0U;
  MotorTest_StopOutputs();
  Wulong_ControlInit();
  Wulong_ControlArm();
  MotorTest_Arm();
  s_vision_last_ms = HAL_GetTick();
  s_vision_valid = 0U;
  g_vision_approach_active = 1U;
  Uart_SendText("VISION APPROACH ON stop_pm=580 max_vx=100 max_omega=350\r\n");
}

static void VisionApproachStop(void)
{
  g_vision_approach_active = 0U;
  Wulong_Stop();
  Wulong_ControlDisarm();
}

static void VisionApproachStep(uint32_t now_ms)
{
  float err;
  float size;
  float turn_norm;
  float vx = 0.0f;
  float omega = 0.0f;
  uint32_t age_ms;

  if (!g_vision_approach_active) {
    return;
  }

  age_ms = now_ms - s_vision_last_ms;
  if (!s_vision_valid || s_vision_conf < VISION_MIN_CONF ||
      age_ms > VISION_TIMEOUT_MS) {
    Wulong_Stop();
    return;
  }

  err = (float)s_vision_err_pm * 0.001f;
  size = (float)s_vision_size_pm * 0.001f;

  if (size >= ((float)VISION_STOP_SIZE_PM * 0.001f) ||
      fabsf((float)s_vision_err_pm) > (float)VISION_FORWARD_ERR_PM) {
    /* At target distance, or while badly off-centre, rotate in place only. */
    vx = 0.0f;
  } else if (size < 0.30f) {
    vx = VISION_MAX_VX_MMPS;
  } else if (size < 0.45f) {
    vx = 75.0f;
  } else {
    vx = 45.0f;
  }

  if (fabsf((float)s_vision_err_pm) > (float)VISION_DEADBAND_PM) {
    turn_norm = (float)s_vision_err_pm / (float)VISION_ALIGN_FULL_PM;
    turn_norm = (turn_norm > 1.0f) ? 1.0f : turn_norm;
    turn_norm = (turn_norm < -1.0f) ? -1.0f : turn_norm;
    /* Positive omega is CCW.  A target to the right needs clockwise yaw. */
    omega = -turn_norm * VISION_MAX_OMEGA_MRADPS;
  }

  vx *= (1.0f - 0.45f * fminf(fabsf(err), 1.0f));
  Wulong_SetBodyVelocityXY(vx, 0.0f, omega);
}

static void Uart_SendU32(uint32_t value)
{
  char buf[11];
  uint8_t n = 0;

  if (value == 0U) {
    Uart_SendText("0");
    return;
  }
  while (value > 0U && n < sizeof(buf)) {
    buf[n++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while (n > 0U) {
    char c = buf[--n];
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1U, 50U);
  }
}

static void Uart_SendS32(int32_t value)
{
  if (value < 0) {
    Uart_SendText("-");
    Uart_SendU32((uint32_t)(-value));
  } else {
    Uart_SendU32((uint32_t)value);
  }
}

static void Uart_SendMotor(uint8_t motor)
{
  char c = (char)('A' + motor);
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1U, 50U);
}

#define IMU_TXT_ORIGINAL       "\xE5\x8E\x9F\xE5\xA7\x8B\xE5\x80\xBC"
#define IMU_TXT_CONVERTED      "\xE6\x8D\xA2\xE7\xAE\x97\xE5\x80\xBC"
#define IMU_TXT_ACCEL          "\xE5\x8A\xA0\xE9\x80\x9F\xE5\xBA\xA6"
#define IMU_TXT_GYRO           "\xE8\xA7\x92\xE9\x80\x9F\xE5\xBA\xA6"
#define IMU_TXT_ATTITUDE       "\xE5\xA7\xBF\xE6\x80\x81\xE8\xA7\x92"
#define IMU_TXT_HEADING        "\xE8\x88\xAA\xE5\x90\x91\xE8\xA7\x92"
#define IMU_TXT_CUMULATIVE     "\xE7\xB4\xAF\xE8\xAE\xA1"
#define IMU_TXT_POWER_ON_X     "\xE4\xB8\x8A\xE7\x94\xB5\xE6\xA0\xA1\xE5\x87\x86\xE6\x97\xB6\xE5\xAE\x9A\xE4\xB9\x89X\xE8\xBD\xB4\xE6\x96\xB9\xE5\x90\x91"
#define IMU_TXT_CCW_POSITIVE   "\xE9\x80\x86\xE6\x97\xB6\xE9\x92\x88\xE4\xB8\xBA\xE6\xAD\xA3"
#define IMU_TXT_BIAS           "\xE9\x9B\xB6\xE5\x81\x8F\xE4\xBC\xB0\xE8\xAE\xA1"
#define IMU_TXT_STATIC         "\xE9\x9D\x99\xE6\xAD\xA2\xE5\x88\xA4\xE5\xAE\x9A"
#define IMU_TXT_STATIC_YES     "\xE6\x98\xAF\x28\xE8\x87\xAA\xE9\x80\x82\xE5\xBA\x94\xE5\xB7\xA5\xE4\xBD\x9C\xE4\xB8\xAD\x29"
#define IMU_TXT_NO             "\xE5\x90\xA6"
#define IMU_TXT_DEGREE         "\xC2\xB0"

static void Uart_SendImuReport(void)
{
  char buf[192];
  float ax = Imu_GetAccXG();
  float ay = Imu_GetAccYG();
  float az = Imu_GetAccZG();
  float cumulative_deg = Imu_GetYawMrad() * 0.001f *
                         (180.0f / 3.14159265358979f);
  float heading_deg = fmodf(cumulative_deg, 360.0f);
  float roll_deg;
  float pitch_deg;

  if (heading_deg < 0.0f) {
    heading_deg += 360.0f;
  }
  roll_deg = Imu_GetRollDeg();
  pitch_deg = Imu_GetPitchDeg();

  (void)snprintf(buf, sizeof(buf),
                 IMU_TXT_ORIGINAL ":  AX %+6d  AY %+6d  AZ %+6d | T %+6d | GX %+6d  GY %+6d  GZ %+6d\r\n",
                 (int)Imu_GetAccXRaw(), (int)Imu_GetAccYRaw(),
                 (int)Imu_GetAccZRaw(), (int)Imu_GetTemperatureRaw(),
                 (int)Imu_GetGyroXRaw(), (int)Imu_GetGyroYRaw(),
                 (int)Imu_GetGyroZRaw());
  Uart_SendText(buf);

  (void)snprintf(buf, sizeof(buf),
                 IMU_TXT_CONVERTED ": " IMU_TXT_ACCEL " %+.3f %+.3f %+.3f g | " IMU_TXT_GYRO " %+.2f %+.2f %+.2f " IMU_TXT_DEGREE "/s | %.1f " IMU_TXT_DEGREE "C\r\n",
                 (double)ax, (double)ay, (double)az,
                 (double)Imu_GetGyroXDps(), (double)Imu_GetGyroYDps(),
                 (double)Imu_GetGyroZDps(), (double)Imu_GetTemperatureC());
  Uart_SendText(buf);

  (void)snprintf(buf, sizeof(buf),
                 IMU_TXT_ATTITUDE ": roll %+.2f" IMU_TXT_DEGREE "  pitch %+.2f" IMU_TXT_DEGREE "\r\n",
                 (double)roll_deg, (double)pitch_deg);
  Uart_SendText(buf);

  (void)snprintf(buf, sizeof(buf),
                 IMU_TXT_HEADING ": %+6.1f" IMU_TXT_DEGREE "   " IMU_TXT_CUMULATIVE ": %+.1f" IMU_TXT_DEGREE "   (0" IMU_TXT_DEGREE "=" IMU_TXT_POWER_ON_X ", " IMU_TXT_CCW_POSITIVE ")\r\n",
                 (double)heading_deg, (double)cumulative_deg);
  Uart_SendText(buf);

  (void)snprintf(buf, sizeof(buf),
                 IMU_TXT_BIAS ": X %+.3f  Y %+.3f  Z %+.3f " IMU_TXT_DEGREE "/s | " IMU_TXT_STATIC ": %s\r\n",
                 (double)Imu_GetGyroXBiasDps(),
                 (double)Imu_GetGyroYBiasDps(),
                 (double)Imu_GetGyroZBiasDps(),
                 Imu_IsStationary() ? IMU_TXT_STATIC_YES : IMU_TXT_NO);
  Uart_SendText(buf);
}

static void MotorTest_HandleCommand(char *line)
{
  char *p;
  uint8_t motor;
  int8_t dir;
  uint32_t a;
  uint32_t b;
  const char *q;

  TrimCommand(line);
  if (line[0] == '\0') {
    return;
  }

  for (p = line; *p != '\0'; ++p) {
    if (*p >= 'a' && *p <= 'z') {
      *p = (char)(*p - 'a' + 'A');
    }
  }

  if (strcmp(line, "H") == 0 || strcmp(line, "?") == 0) {
    Uart_SendText("\r\nCommands:\r\n");
    Uart_SendText("  ARM / DISARM / S(stop+disarm) / E(estop)\r\n");
    Uart_SendText("  P = motor A forward 10% for 800 ms\r\n");
    Uart_SendText("  F = motor A FULL forward 100% for 2000 ms\r\n");
    Uart_SendText("  L = ALL wheels FULL forward 100% for 2000 ms\r\n");
    Uart_SendText("  N = motor A reverse 10% for 800 ms\r\n");
    Uart_SendText("  1/2=A +/-  3/4=B +/-  5/6=C +/-  7/8=D +/-\r\n");
    Uart_SendText("  K1 button = start K230 vision approach\r\n");
    Uart_SendText("  K1 serial = FWD 2s -> STRAFE_L 2s -> STRAFE_R 2s -> BWD 2s\r\n");
    Uart_SendText("  K2 = FORWARD 0->100% in 5 s, hold 100% for 10 s\r\n");
    Uart_SendText("  R  = turn RIGHT 20% for 1000 ms\r\n");
    Uart_SendText("  K3 = gyro calibrate -> random pivot 5s -> heading + mecanum XY return\r\n");
    Uart_SendText("  STUNE = auto step test 100/150/200 mm/s on four wheels\r\n");
    Uart_SendText("  T = auto test A+ A- B+ B- C+ C- D+ D-\r\n");
    Uart_SendText("  W = drive all wheels forward 20%\r\n");
    Uart_SendText("  X = drive all wheels reverse 20%\r\n");
    Uart_SendText("  Q = print encoder counts (A B C D)\r\n");
    Uart_SendText("  A+ [duty_ms] | A+ <permille> <ms>   (also B/C/D, +/-)\r\n");
    Uart_SendText("  RUN <A-D> <+/-/F/R> <permille> <ms>\r\n");
    Uart_SendText("  ENC | Z(reset encoders) | V(battery mV) | STAT\r\n");
    Uart_SendText("  permille: 1..300 (10=1%), ms: 1..5000\r\n");
    Uart_SendText("  Multi-char commands execute on CR/LF or semicolon.\r\n");
    Uart_SendText("  Commands execute on CR/LF, semicolon, or 150 ms idle.\r\n\r\n");
    Uart_SendText(" Vision approach (K230 over USART2):\r\n");
    Uart_SendText("  VA            enable water-can visual approach\r\n");
    Uart_SendText("  VS            stop visual approach and disarm\r\n");
    Uart_SendText("  VI            print vision receiver status\r\n\r\n");
    Uart_SendText(" Calibration console (closed-loop speed control):\r\n");
    Uart_SendText("  CV            toggle console on/off\r\n");
    Uart_SendText("  CS <0-3> <mmps>       set one wheel target speed\r\n");
    Uart_SendText("  CSA <mmps>            set all four wheels in one tick\r\n");
    Uart_SendText("  CSV <w0> <w1> <w2> <w3>  set four targets in one line\r\n");
    Uart_SendText("  FLIP <0-3> [0|1]  wheel mounted the other way round\r\n");
    Uart_SendText("  CG <0-3> <kp> <ki> <kd>  set speed-loop gains\r\n");
    Uart_SendText("  CH <mm>       set half-diagonal lever arm\r\n");
    Uart_SendText("  CM <val>      set mm per encoder count\r\n");
    Uart_SendText("  YG <0.30-1.19> set mecanum lateral ground/wheel gain\r\n");
    Uart_SendText("  CP            print wheel speeds, pose, calibration\r\n");
    Uart_SendText("  CZ            zero pose and encoders\r\n");
    Uart_SendText("  II            print IMU status\r\n");
    Uart_SendText("  IC            calibrate gyro bias (keep still)\r\n");
    Uart_SendText("  ECHO          mirror received bytes back (RX/TX check)\r\n");
    Uart_SendText("  IT [div]      telemetry on/off, motors stay disarmed\r\n");
    Uart_SendText("                div = print every <div> ms (default 10)\r\n\r\n");
    Uart_SendText("  IR [div]      formatted IMU report on/off (default 2000 ms)\r\n\r\n");
    return;
  }

  /*
   * While telemetry is streaming, only allow commands that cannot energise a
   * motor.  This keeps the "watched channel + hand-turned chassis" test safe.
   * CV is allowed because it turns telemetry off and switches modes.
   */
  if ((g_imu_telemetry || g_imu_report) &&
      !IsTelemetrySafeCommand(line)) {
    Uart_SendText("ERR telemetry on: send IT to stop\r\n");
    return;
  }

  if ((k3_test_active || speed_tune_active) &&
      strcmp(line, "S") != 0 && strcmp(line, "STOP") != 0 &&
      strcmp(line, "E") != 0 && strcmp(line, "ESTOP") != 0 &&
      strcmp(line, "DISARM") != 0 && strcmp(line, "Q") != 0 &&
      strcmp(line, "ENC") != 0 && strcmp(line, "V") != 0 &&
      strcmp(line, "STAT") != 0 && strcmp(line, "II") != 0 &&
      strcmp(line, "ECHO") != 0) {
    Uart_SendText("ERR auto test busy: send S to abort\r\n");
    return;
  }

  if (strcmp(line, "P") == 0 || strcmp(line, "1") == 0) {
    MotorTest_RunSingle(0U, 1);
    return;
  }

  if (strcmp(line, "F") == 0) {
    MotorTest_RunFullA();
    return;
  }

  if (strcmp(line, "L") == 0) {
    MotorTest_RunAllFull();
    return;
  }

  if (strcmp(line, "R") == 0) {
    MotorTest_TurnRightStart();
    return;
  }

  if (strcmp(line, "W") == 0) {
    MotorTest_DriveAll(1);
    return;
  }

  if (strcmp(line, "X") == 0) {
    MotorTest_DriveAll(-1);
    return;
  }

  if (strcmp(line, "N") == 0 || strcmp(line, "2") == 0) {
    MotorTest_RunSingle(0U, -1);
    return;
  }

  if (strcmp(line, "3") == 0) {
    MotorTest_RunSingle(1U, 1);
    return;
  }

  if (strcmp(line, "4") == 0) {
    MotorTest_RunSingle(1U, -1);
    return;
  }

  if (strcmp(line, "5") == 0) {
    MotorTest_RunSingle(2U, 1);
    return;
  }

  if (strcmp(line, "6") == 0) {
    MotorTest_RunSingle(2U, -1);
    return;
  }

  if (strcmp(line, "7") == 0) {
    MotorTest_RunSingle(3U, 1);
    return;
  }

  if (strcmp(line, "8") == 0) {
    MotorTest_RunSingle(3U, -1);
    return;
  }

  if (strcmp(line, "T") == 0) {
    MotorTest_AutoStart();
    return;
  }

  if (strcmp(line, "Q") == 0) {
    Uart_SendText("ENC A="); Uart_SendS32(MotorTest_ReadEncoder(0));
    Uart_SendText(" B=");   Uart_SendS32(MotorTest_ReadEncoder(1));
    Uart_SendText(" C=");   Uart_SendS32(MotorTest_ReadEncoder(2));
    Uart_SendText(" D=");   Uart_SendS32(MotorTest_ReadEncoder(3));
    Uart_SendText("\r\n");
    return;
  }

  /*
   * FLIP <wheel 0-3> [0|1] - declare that wheel "mounted the other way round".
   *
   * It inverts that wheel's direction lines AND negates its encoder feedback.
   * Doing only one of the two leaves the encoder reading positive while the
   * wheel spins backwards, which the per-channel sign test cannot detect - only
   * driving the robot can.  Without an argument the command toggles.
   */
  if (strncmp(line, "FLIP ", 5) == 0) {
    const char *p = SkipSpaces(line + 5);
    uint32_t wheel;
    uint32_t value;
    uint8_t on;

    if (!ParseU32(&p, &wheel) || wheel > 3U) {
      Uart_SendText("ERR FLIP <0-3> [0|1]\r\n");
      return;
    }
    p = SkipSpaces(p);
    if (*p == '\0') {
      on = (uint8_t)(Wulong_GetWheelFlip((Wulong_Wheel)wheel) ? 0U : 1U);
    } else if (ParseU32(&p, &value) && value <= 1U) {
      on = (uint8_t)value;
    } else {
      Uart_SendText("ERR FLIP <0-3> [0|1]\r\n");
      return;
    }
    Wulong_SetWheelFlip((Wulong_Wheel)wheel, on);
    Uart_SendText("FLIP wheel="); Uart_SendU32(wheel);
    Uart_SendText(" flipped="); Uart_SendU32(on);
    Uart_SendText("\r\n");
    return;
  }

  /* Same four-phase sequence as the K1 button, so it can be run from the
   * serial console when the board buttons are not reachable. */
  if (strcmp(line, "K1") == 0 || strcmp(line, "SEQ") == 0) {
    MotorTest_K1SeqStart();
    return;
  }

  if (strcmp(line, "K2") == 0) {
    MotorTest_K2RampStart();
    return;
  }

  if (strcmp(line, "K3") == 0) {
    MotorTest_K3Start();
    return;
  }

  if (strcmp(line, "STUNE") == 0) {
    MotorTest_SpeedTuneStart();
    return;
  }

  /* ---- K230 vision approach ----------------------------------------- */

  if (strcmp(line, "VA") == 0) {
    VisionApproachStart();
    return;
  }

  if (strcmp(line, "VS") == 0) {
    VisionApproachStop();
    MotorTest_Disarm();
    Uart_SendText("VISION APPROACH OFF\r\n");
    return;
  }

  if (strcmp(line, "VI") == 0) {
    uint32_t age_ms = HAL_GetTick() - s_vision_last_ms;

    Uart_SendText("VIS active=");
    Uart_SendU32((uint32_t)g_vision_approach_active);
    Uart_SendText(" valid=");
    Uart_SendText(s_vision_valid ? "1" : "0");
    Uart_SendText(" err_pm=");
    Uart_SendS32((int32_t)s_vision_err_pm);
    Uart_SendText(" size_pm=");
    Uart_SendS32((int32_t)s_vision_size_pm);
    Uart_SendText(" conf=");
    Uart_SendU32((uint32_t)s_vision_conf);
    Uart_SendText(" age_ms=");
    Uart_SendU32(age_ms);
    Uart_SendText(" frames=");
    Uart_SendU32(s_vision_frame_count);
    Uart_SendText(" rx_bytes=");
    Uart_SendU32(s_k230_rx_count);
    Uart_SendText(" crc_err=");
    Uart_SendU32(s_vision_crc_errors);
    Uart_SendText("\r\n");
    return;
  }

  /* ---- Calibration console ------------------------------------------- */

  if (strcmp(line, "CV") == 0) {
    g_vision_approach_active = 0U;
    MotorTest_AutoCancel();
    k1_seq_active = 0U;
    k2_ramp_active = 0U;
    turn_right_active = 0U;
    drive_mode_active = 0U;
    MotorTest_StopOutputs();
    g_ctrl_console_active = (uint8_t)(g_ctrl_console_active ? 0U : 1U);
    if (g_ctrl_console_active) {
      g_imu_telemetry = 0U;
      g_imu_report = 0U;
      Wulong_ControlInit();
      Uart_SendText("CTRL CONSOLE ON (speed loop 1kHz)\r\n");
    } else {
      Wulong_ControlDisarm();
      Uart_SendText("CTRL CONSOLE OFF\r\n");
    }
    return;
  }

  if (strncmp(line, "CS ", 3) == 0) {
    uint32_t wheel;
    float mmps;

    q = line + 3;
    if (!ParseU32(&q, &wheel) || !ParseFloat(&q, &mmps) || wheel > 3U) {
      Uart_SendText("ERR CS <0-3> <mmps>\r\n");
      return;
    }
    if (!g_ctrl_console_active) {
      Uart_SendText("ERR send CV first\r\n");
      return;
    }
    Wulong_SetWheelTarget((Wulong_Wheel)wheel, mmps);
    Uart_SendText("CS wheel="); Uart_SendU32(wheel);
    Uart_SendText(" target_mmps="); Uart_SendU32((uint32_t)fabsf(mmps));
    Uart_SendText("\r\n");
    return;
  }

  /* Set all four wheels from ONE command line, so they start in the same
   * control tick.  Issuing four separate CS lines staggers the start by the
   * inter-command delay (measured: 0 / 64 / 126 / 189 ms). */
  if (strncmp(line, "CSA ", 4) == 0) {
    float mmps;
    uint8_t i;

    q = line + 4;
    if (!ParseFloat(&q, &mmps)) {
      Uart_SendText("ERR CSA <mmps>\r\n");
      return;
    }
    if (!g_ctrl_console_active) {
      Uart_SendText("ERR send CV first\r\n");
      return;
    }
    for (i = 0U; i < 4U; ++i) {
      Wulong_SetWheelTarget((Wulong_Wheel)i, mmps);
    }
    Uart_SendText("CSA all target_mmps="); Uart_SendU32((uint32_t)fabsf(mmps));
    Uart_SendText("\r\n");
    return;
  }

  if (strncmp(line, "CSV ", 4) == 0) {
    float v[4];
    uint8_t i;

    q = line + 4;
    for (i = 0U; i < 4U; ++i) {
      if (!ParseFloat(&q, &v[i])) {
        Uart_SendText("ERR CSV <w0> <w1> <w2> <w3>\r\n");
        return;
      }
    }
    if (!g_ctrl_console_active) {
      Uart_SendText("ERR send CV first\r\n");
      return;
    }
    for (i = 0U; i < 4U; ++i) {
      Wulong_SetWheelTarget((Wulong_Wheel)i, v[i]);
    }
    Uart_SendText("CSV ");
    for (i = 0U; i < 4U; ++i) {
      Uart_SendText(" wheel="); Uart_SendU32(i);
      Uart_SendText(":"); Uart_SendU32((uint32_t)fabsf(v[i]));
    }
    Uart_SendText("\r\n");
    return;
  }

  if (strncmp(line, "CG ", 3) == 0) {
    uint32_t wheel;
    float kp, ki, kd;

    q = line + 3;
    if (!ParseU32(&q, &wheel) || !ParseFloat(&q, &kp) ||
        !ParseFloat(&q, &ki) || !ParseFloat(&q, &kd) || wheel > 3U) {
      Uart_SendText("ERR CG <0-3> <kp> <ki> <kd>\r\n");
      return;
    }
    Wulong_SetWheelGains((Wulong_Wheel)wheel, kp, ki, kd);
    Uart_SendText("CG OK\r\n");
    return;
  }

  if (strncmp(line, "CH ", 3) == 0) {
    float v;

    q = line + 3;
    if (!ParseFloat(&q, &v)) {
      Uart_SendText("ERR CH <half_diag_mm>\r\n");
      return;
    }
    Wulong_SetHalfDiag(v);
    Uart_SendText("CH half_diag="); Uart_SendU32((uint32_t)Wulong_GetHalfDiag());
    Uart_SendText("\r\n");
    return;
  }

  if (strncmp(line, "CM ", 3) == 0) {
    float v;
    char buf[16];

    q = line + 3;
    if (!ParseFloat(&q, &v)) {
      Uart_SendText("ERR CM <mm_per_count>\r\n");
      return;
    }
    Wulong_SetMmPerCount(v);
    (void)snprintf(buf, sizeof(buf), "CM %.6f\r\n", (double)Wulong_GetMmPerCount());
    Uart_SendText(buf);
    return;
  }

  if (strncmp(line, "YG ", 3) == 0) {
    float v;
    char buf[32];

    q = line + 3;
    if (!ParseFloat(&q, &v) || v <= 0.3f || v >= 1.2f) {
      Uart_SendText("ERR YG <0.30-1.19>\r\n");
      return;
    }
    Wulong_SetLateralGain(v);
    (void)snprintf(buf, sizeof(buf), "YG %.3f\r\n",
                   (double)Wulong_GetLateralGain());
    Uart_SendText(buf);
    return;
  }

  if (strcmp(line, "CP") == 0) {
    char buf[96];
    Wulong_Pose pose = Wulong_GetPose();

    (void)snprintf(buf, sizeof(buf),
                   "SPD %d %d %d %d mm/s\r\n",
                   (int)Wulong_GetWheelSpeed(WULONG_FL),
                   (int)Wulong_GetWheelSpeed(WULONG_RL),
                   (int)Wulong_GetWheelSpeed(WULONG_RR),
                   (int)Wulong_GetWheelSpeed(WULONG_FR));
    Uart_SendText(buf);
    (void)snprintf(buf, sizeof(buf),
                   "POSE x=%d y=%d th=%d mrad\r\n",
                   (int)pose.x_mm, (int)pose.y_mm, (int)pose.theta_mrad);
    Uart_SendText(buf);
    (void)snprintf(buf, sizeof(buf),
                   "CAL d=%d um/count=%d\r\n",
                   (int)Wulong_GetHalfDiag(),
                   (int)(Wulong_GetMmPerCount() * 1000000.0f));
    Uart_SendText(buf);
    (void)snprintf(buf, sizeof(buf),
                   "ENCQ rejects=%u (encoder samples dropped as impossible)\r\n",
                   (unsigned int)Wulong_GetEncoderRejectCount());
    Uart_SendText(buf);
    (void)snprintf(buf, sizeof(buf),
                   "IMU ready=%u cal=%u bias=%.1f yaw=%d gz=%.1f\r\n",
                   (unsigned int)Imu_IsReady(), (unsigned int)Imu_CalibrationDone(),
                   (double)Imu_GetGyroZBiasRaw(), (int)Imu_GetYawMrad(),
                   (double)Imu_GetGyroZDps());
    Uart_SendText(buf);
    return;
  }

  if (strcmp(line, "II") == 0) {
    char buf[160];

    (void)snprintf(buf, sizeof(buf),
                   "IMU ready=%u cal=%u samples=%u bias=%.1f noise=%.3fdps deadband=%.3fdps yaw=%d gz=%.1f\r\n",
                   (unsigned int)Imu_IsReady(), (unsigned int)Imu_CalibrationDone(),
                   (unsigned int)Imu_GetCalibrationSamples(), (double)Imu_GetGyroZBiasRaw(),
                   (double)Imu_GetCalibrationNoiseDps(),
                   (double)Imu_GetStationaryDeadbandDps(),
                   (int)Imu_GetYawMrad(), (double)Imu_GetGyroZDps());
    Uart_SendText(buf);
    return;
  }

  if (strcmp(line, "IC") == 0) {
    if (!Imu_IsReady()) {
      Uart_SendText("ERR IMU not ready\r\n");
      return;
    }
    g_imu_auto_cal_suppressed = 1U;
    Imu_StartBiasCalibration(2000U);
    Uart_SendText("IMU BIAS CAL START (2000 samples, keep still)\r\n");
    return;
  }

  if (strcmp(line, "IT") == 0 || strncmp(line, "IT ", 3) == 0) {
    if (strcmp(line, "IT") == 0) {
      g_imu_telemetry = (uint8_t)(g_imu_telemetry ? 0U : 1U);
    } else {
      uint32_t div;

      q = line + 3;
      if (!ParseU32(&q, &div) || div < 1U || div > 1000U) {
        Uart_SendText("ERR IT [1-1000 ms]\r\n");
        return;
      }
      g_imu_telemetry_div = (uint16_t)div;
      g_imu_telemetry = 1U;
    }

    if (g_imu_telemetry) {
      /* Leave the closed-loop console too: its 1 kHz speed loop would hold the
       * wheels at zero speed and load the chassis while it is turned by hand. */
      g_imu_report = 0U;
      g_ctrl_console_active = 0U;
      g_vision_approach_active = 0U;
      Wulong_ControlDisarm();
      MotorTest_AutoCancel();
      k1_seq_active = 0U;
      k2_ramp_active = 0U;
      turn_right_active = 0U;
      drive_mode_active = 0U;
      all_full_active = 0U;
      MotorTest_StopOutputs();
      MotorTest_Disarm();
      Uart_SendText("IMU TELEMETRY ON (motors disarmed), div=");
      Uart_SendU32(g_imu_telemetry_div);
      Uart_SendText(" ms\r\n");
    } else {
      Uart_SendText("IMU TELEMETRY OFF\r\n");
    }
    return;
  }

  if (strcmp(line, "IR") == 0 || strncmp(line, "IR ", 3) == 0) {
    if (strcmp(line, "IR") == 0) {
      g_imu_report = (uint8_t)(g_imu_report ? 0U : 1U);
    } else {
      uint32_t div;

      q = line + 3;
      if (!ParseU32(&q, &div) || div < 10U || div > 5000U) {
        Uart_SendText("ERR IR [10-5000 ms]\r\n");
        return;
      }
      g_imu_report_div = (uint16_t)div;
      g_imu_report = 1U;
    }

    if (g_imu_report) {
      g_imu_telemetry = 0U;
      g_ctrl_console_active = 0U;
      g_vision_approach_active = 0U;
      Wulong_ControlDisarm();
      MotorTest_AutoCancel();
      k1_seq_active = 0U;
      k2_ramp_active = 0U;
      turn_right_active = 0U;
      drive_mode_active = 0U;
      all_full_active = 0U;
      MotorTest_StopOutputs();
      MotorTest_Disarm();
      Uart_SendText("IMU REPORT ON (motors disarmed), div=");
      Uart_SendU32(g_imu_report_div);
      Uart_SendText(" ms\r\n");
    } else {
      Uart_SendText("IMU REPORT OFF\r\n");
    }
    return;
  }

  if (strcmp(line, "CZ") == 0) {
    Wulong_ResetPose(0.0f, 0.0f, 0.0f);
    Uart_SendText("POSE ZEROED\r\n");
    return;
  }

  if (strcmp(line, "S") == 0 || strcmp(line, "STOP") == 0) {
    if (k3_test_active) {
      MotorTest_K3Finish("STOP");
      return;
    }
    if (speed_tune_active) {
      MotorTest_SpeedTuneFinish("STOP");
      return;
    }
    MotorTest_AutoCancel();
    k1_seq_active = 0U;
    g_ctrl_console_active = 0U;
    g_vision_approach_active = 0U;
    Wulong_ControlDisarm();
    MotorTest_Disarm();
    Uart_SendText("STOPPED + DISARMED\r\n");
    return;
  }

  if (strcmp(line, "E") == 0 || strcmp(line, "ESTOP") == 0) {
    if (k3_test_active) {
      MotorTest_K3Finish("ESTOP");
      return;
    }
    if (speed_tune_active) {
      MotorTest_SpeedTuneFinish("ESTOP");
      return;
    }
    MotorTest_AutoCancel();
    k1_seq_active = 0U;
    g_ctrl_console_active = 0U;
    g_vision_approach_active = 0U;
    Wulong_ControlDisarm();
    MotorTest_Disarm();
    Uart_SendText("EMERGENCY STOP\r\n");
    return;
  }

  if (strcmp(line, "ARM") == 0) {
    MotorTest_Arm();
    Uart_SendText("ARMED\r\n");
    return;
  }

  if (strcmp(line, "DISARM") == 0) {
    if (k3_test_active) {
      MotorTest_K3Finish("DISARM");
      return;
    }
    if (speed_tune_active) {
      MotorTest_SpeedTuneFinish("DISARM");
      return;
    }
    MotorTest_AutoCancel();
    k1_seq_active = 0U;
    g_vision_approach_active = 0U;
    MotorTest_Disarm();
    Uart_SendText("DISARMED\r\n");
    return;
  }

  if (strcmp(line, "Z") == 0) {
    MotorTest_ResetEncoders();
    Uart_SendText("ENC ZEROED\r\n");
    return;
  }

  if (strcmp(line, "ENC") == 0) {
    Uart_SendText("ENC A="); Uart_SendS32(MotorTest_ReadEncoder(0));
    Uart_SendText(" B=");   Uart_SendS32(MotorTest_ReadEncoder(1));
    Uart_SendText(" C=");   Uart_SendS32(MotorTest_ReadEncoder(2));
    Uart_SendText(" D=");   Uart_SendS32(MotorTest_ReadEncoder(3));
    Uart_SendText("\r\n");
    return;
  }

  if (strcmp(line, "V") == 0) {
    Uart_SendText("VBAT_mV=");
    Uart_SendU32(MotorTest_ReadBatteryMv());
    Uart_SendText("\r\n");
    return;
  }

  if (strcmp(line, "STAT") == 0) {
    Uart_SendText("armed="); Uart_SendU32(g_armed);
    Uart_SendText(" active="); Uart_SendU32((uint32_t)active_dir);
    Uart_SendText(" t_ms="); Uart_SendU32(HAL_GetTick());
    Uart_SendText(" rx="); Uart_SendU32(s_cli_rx_count);
    Uart_SendText(" k3="); Uart_SendText(k3_last_result);
    Uart_SendText(" stune="); Uart_SendU32(speed_tune_done);
    Uart_SendText("/"); Uart_SendU32(speed_tune_phase);
    Uart_SendText("\r\n");
    return;
  }

  if (strcmp(line, "ECHO") == 0) {
    g_cli_echo = (uint8_t)(g_cli_echo ? 0U : 1U);
    Uart_SendText(g_cli_echo ? "ECHO ON\r\n" : "ECHO OFF\r\n");
    return;
  }

  if (strncmp(line, "RUN ", 4) == 0) {
    q = line + 4;
    if (!ParseMotor(&q, &motor)) {
      Uart_SendText("ERR motor A-D\r\n");
      return;
    }
    q = SkipSpaces(q);
    if (*q == '+' || *q == 'F') {
      dir = 1;
    } else if (*q == '-' || *q == 'R') {
      dir = -1;
    } else {
      Uart_SendText("ERR dir + - F R\r\n");
      return;
    }
    ++q;
    if (!ParseU32(&q, &a) || !ParseU32(&q, &b)) {
      Uart_SendText("ERR RUN <M> <dir> <permille> <ms>\r\n");
      return;
    }
    if (!g_armed) {
      Uart_SendText("ERR not armed: send ARM first\r\n");
      return;
    }
    MotorTest_Start(motor, dir, (uint16_t)a, b);
    Uart_SendText("RUN ");
    Uart_SendMotor(motor);
    Uart_SendText(dir > 0 ? "+" : "-");
    Uart_SendText(" duty="); Uart_SendU32(a);
    Uart_SendText(" ms="); Uart_SendU32(b);
    Uart_SendText("\r\n");
    return;
  }

  if (line[0] >= 'A' && line[0] <= 'D' &&
      (line[1] == '+' || line[1] == '-')) {
    motor = (uint8_t)(line[0] - 'A');
    dir = (line[1] == '+') ? 1 : -1;
    q = line + 2;
    a = 100U;
    b = 800U;

    q = SkipSpaces(q);
    if (*q != '\0') {
      if (!ParseU32(&q, &a)) {
        Uart_SendText("ERR quick: A+ [ms] or A+ <permille> <ms>\r\n");
        return;
      }
      q = SkipSpaces(q);
      if (*q != '\0') {
        if (!ParseU32(&q, &b)) {
          Uart_SendText("ERR quick: A+ <permille> <ms>\r\n");
          return;
        }
      } else {
        b = a;
        a = 100U;
      }
    }

    MotorTest_Arm();
    MotorTest_Start(motor, dir, (uint16_t)a, b);
    Uart_SendText("QUICK ");
    Uart_SendMotor(motor);
    Uart_SendText(dir > 0 ? "+" : "-");
    Uart_SendText(" duty="); Uart_SendU32(a);
    Uart_SendText(" ms="); Uart_SendU32(b);
    Uart_SendText("\r\n");
    return;
  }

  Uart_SendText("ERR RX=\"");
  Uart_SendText(line);
  Uart_SendText("\" - type H for help\r\n");
}

static uint8_t IsTelemetrySafeCommand(const char *line)
{
  static const char *const allowed[] = {
    "S", "STOP", "E", "ESTOP", "DISARM", "Q", "ENC", "Z",
    "V", "STAT", "II", "IC", "IT", "IR", "CV", "CP", "CZ", "ECHO",
    "VS", "VI"
  };
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(allowed) / sizeof(allowed[0])); ++i) {
    if (strcmp(line, allowed[i]) == 0) {
      return 1U;
    }
  }
  if (strncmp(line, "IT ", 3) == 0 || strncmp(line, "IR ", 3) == 0) {
    return 1U;
  }
  return 0U;
}

/*
 * Dedicated IMU task.
 *
 * Keeping the 1 kHz gyro sampling out of ControlTask means:
 *   - the IMU keeps running in the plain bring-up mode, not just inside CV;
 *   - calibration and yaw integration continue while the telemetry channel is
 *     being watched and the chassis is turned by hand.
 */
static void ImuTask(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);
  uint32_t last_print_ms = 0U;
  uint32_t last_report_ms = 0U;
  uint32_t last_tick_ms = HAL_GetTick();
  uint32_t task_start_ms = last_tick_ms;
  uint8_t  auto_cal_started = 0U;
  uint8_t  auto_cal_reported = 0U;

  (void) argument;

  for (;;) {
    uint32_t now = HAL_GetTick();
    float dt_s = (float)(now - last_tick_ms) * 0.001f;

    if (dt_s > 0.0f) {
      last_tick_ms = now;
      Imu_Update(dt_s);
    }

    /*
     * Auto-run the stationary bias calibration once, shortly after boot.
     * Without it bias_raw stays 0 and Imu_Update() refuses to integrate yaw,
     * so a freshly flashed board would appear completely dead while still
     * reporting ready=1.
     *
     * The 10 s warm-up lets the sensor self-heating stabilise before bias
     * calibration.  A manual IC command can still re-trigger calibration.
     */
    if (!auto_cal_started && !g_imu_auto_cal_suppressed && Imu_IsReady() &&
        (uint32_t)(now - task_start_ms) >= 10000U) {
      auto_cal_started = 1U;
      Imu_StartBiasCalibration(2000U);
    }

    if (auto_cal_started && !auto_cal_reported && Imu_CalibrationDone()) {
      char buf[80];

      auto_cal_reported = 1U;
      (void)snprintf(buf, sizeof(buf),
                     "IMU AUTO BIAS DONE biasZ=%d spread=%.3f accRef=%.4f\r\n",
                     (int)Imu_GetGyroZBiasRaw(),
                     (double)Imu_GetCalibrationSpreadDps(),
                     (double)Imu_GetAccMagReference());
      Uart_SendText(buf);
    }

    if (g_imu_telemetry && !g_cli_busy &&
        (uint32_t)(now - last_print_ms) >= (uint32_t)g_imu_telemetry_div) {
      char buf[112];

      last_print_ms = now;
      (void)snprintf(buf, sizeof(buf),
                     "IMU %d %d %d %d %.2f cal=%u\r\n",
                     (int)now,
                     (int)Imu_GetGyroZRaw(),
                     (int)Imu_GetGyroZBiasRaw(),
                     (int)Imu_GetYawMrad(),
                     (double)Imu_GetGyroZDps(),
                     (unsigned int)Imu_CalibrationDone());
      Uart_SendText(buf);
    }

    if (g_imu_report && !g_cli_busy &&
        (uint32_t)(now - last_report_ms) >= (uint32_t)g_imu_report_div) {
      last_report_ms = now;
      Uart_SendImuReport();
    }

    vTaskDelayUntil(&last_wake, period);
  }
}

static uint8_t MotorTest_DipSwitchMode(void)
{
  static uint32_t last_start_ms = 0;
  static uint8_t  last_dip1 = 1;
  static uint8_t  last_dip2 = 1;
  uint8_t dip1;
  uint8_t dip2;
  int8_t  dir;

  /* DIP switches are active-low with internal pull-ups. */
  dip1 = (HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  dip2 = (HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (dip1) {
    dir = dip2 ? 1 : -1;

    if (last_dip1 || last_dip2 != dip2 ||
        (uint32_t)(HAL_GetTick() - last_start_ms) >= 3000U) {
      Uart_SendText("DIP TEST ");
      Uart_SendText("A");
      Uart_SendText(dir > 0 ? "+" : "-");
      Uart_SendText(" duty=100 ms=800\r\n");
      MotorTest_Start(0U, dir, 100U, 800U);
      last_start_ms = HAL_GetTick();
    }
  }

  last_dip1 = dip1;
  last_dip2 = dip2;
  return dip1;
}

static void ControlTask(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);
  uint32_t now;

  (void) argument;

  for (;;)
  {
    now = HAL_GetTick();

    if (g_k3_test_request) {
      g_k3_test_request = 0U;
      MotorTest_K3Start();
    }
    if (g_speed_tune_request) {
      g_speed_tune_request = 0U;
      MotorTest_SpeedTuneStart();
    }
    if (g_k2_test_request) {
      g_k2_test_request = 0U;
      if (!k3_test_active && !speed_tune_active &&
          !g_ctrl_console_active && !g_imu_telemetry) {
        MotorTest_K2RampStart();
      }
    }

    /*
     * Calibration console mode: run the closed-loop chassis stack at 1 kHz
     * instead of the bring-up test state machines.
     */
    if (g_ctrl_console_active || g_vision_approach_active ||
        k3_test_active || speed_tune_active) {
      if (g_estop_request) {
        g_estop_request = 0;
        if (k3_test_active) {
          MotorTest_K3Finish("ESTOP");
        } else if (speed_tune_active) {
          MotorTest_SpeedTuneFinish("ESTOP");
        } else if (g_vision_approach_active) {
          VisionApproachStop();
          Uart_SendText("ESTOP (vision off)\r\n");
        } else {
          g_ctrl_console_active = 0U;
          Wulong_ControlDisarm();
          Uart_SendText("ESTOP (console off)\r\n");
        }
      } else {
        VisionApproachStep(now);
        Wulong_ControlTick();
        MotorTest_K3Step();
        MotorTest_SpeedTuneStep();
      }
      vTaskDelayUntil(&last_wake, period);
      continue;
    }

    if (!g_imu_telemetry && !g_imu_report) {
      MotorTest_KeyScan();
      (void)MotorTest_DipSwitchMode();
    }

    if (g_estop_request) {
      g_estop_request = 0;
      MotorTest_AutoCancel();
      k1_seq_active = 0U;
      k2_ramp_active = 0U;
      turn_right_active = 0U;
          MotorTest_Disarm();
      Uart_SendText("ESTOP\r\n");
    }

    if (g_test_trigger) {
      g_test_trigger = 0;
      if (!k1_seq_active) {
        MotorTest_Arm();
        MotorTest_Start((uint8_t)g_test_motor,
                        (int8_t)g_test_dir,
                        (uint16_t)g_test_duty_permille,
                        (uint32_t)g_test_duration_ms);
        Uart_SendText("WATCH TEST START\r\n");
      }
    }

    MotorTest_RunAllFullStep();

    if (!auto_test_active && !drive_mode_active && !k1_seq_active &&
        !k2_ramp_active && !turn_right_active && !all_full_active &&
        active_dir != 0 &&
        (int32_t)(now - active_until_ms) >= 0) {
      MotorTest_Disarm();
      Uart_SendText("TIMEOUT STOP\r\n");
    }

    MotorTest_AutoStep();
    MotorTest_K1SeqStep();
    MotorTest_K2RampStep();
    MotorTest_TurnRightStep();

    vTaskDelayUntil(&last_wake, period);
  }
}

static void CommsTask(void *argument)
{
  char line[64];
  uint8_t line_len = 0;
  uint8_t rx_byte = 0;
  uint32_t last_rx_ms = 0U;

  (void) argument;

  for (;;)
  {
    uint32_t now = HAL_GetTick();

    while (K230GetByte(&rx_byte)) {
      K230ParserFeed(rx_byte);
    }

    while (CliGetByte(&rx_byte)) {
      last_rx_ms = now;
      if (g_cli_echo) {
        /* ECHO: mirror the raw byte back so TX and RX wiring can be verified. */
        (void)HAL_UART_Transmit(&huart1, &rx_byte, 1U, 10U);
      }
      if (rx_byte == '\r' || rx_byte == '\n') {
        if (line_len > 0U) {
          line[line_len] = '\0';
          g_cli_busy = 1U;
          MotorTest_HandleCommand(line);
          g_cli_busy = 0U;
          line_len = 0U;
        }
      } else if (rx_byte == ';') {
        if (line_len > 0U) {
          line[line_len] = '\0';
          g_cli_busy = 1U;
          MotorTest_HandleCommand(line);
          g_cli_busy = 0U;
          line_len = 0U;
        }
      } else if (line_len == 0U && (rx_byte == ' ' || rx_byte == '\t')) {
        /* Ignore leading whitespace before a command. */
      } else if (line_len < (sizeof(line) - 1U)) {
        line[line_len++] = (char)rx_byte;
      } else {
        /* Line buffer full: flush what we have rather than silently dropping
         * the rest and leaving the command stuck forever. */
        line[line_len] = '\0';
        g_cli_busy = 1U;
        MotorTest_HandleCommand(line);
        g_cli_busy = 0U;
        line_len = 0U;
      }
    }

    /*
     * Auto-execute after ~150 ms of line idle.  Many serial terminals do not
     * append CR/LF, so without this a two-character command such as II or IC
     * would sit in the buffer forever and look like "no response".
     */
    if (line_len > 0U && (uint32_t)(now - last_rx_ms) >= 150U) {
      line[line_len] = '\0';
      g_cli_busy = 1U;
      MotorTest_HandleCommand(line);
      g_cli_busy = 0U;
      line_len = 0U;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void StrategyTask(void *argument)
{
  (void) argument;

  for (;;)
  {
    /* State machine and path planning go here. */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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

#ifdef  USE_FULL_ASSERT
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
