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
#include <string.h>
#include <stdio.h>
#include <math.h>

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

/* USER CODE BEGIN PV */
static TaskHandle_t control_task_handle;
static TaskHandle_t comms_task_handle;
static TaskHandle_t strategy_task_handle;

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

static volatile uint8_t  active_motor = 0;
static volatile int8_t   active_dir = 0;
static volatile uint32_t active_until_ms = 0;
static volatile uint8_t  drive_mode_active = 0;
static volatile uint8_t  square_test_active = 0;
static volatile uint8_t  turn_left_active = 0;
static volatile uint8_t  turn_right_active = 0;

/* When set, the control task runs the closed-loop chassis stack
 * (Wulong_ControlTick) instead of the bring-up test state machines. */
static volatile uint8_t  g_ctrl_console_active = 0;

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

static void MotorTest_Init(void);
static void MotorTest_Arm(void);
static void MotorTest_Disarm(void);
static void MotorTest_SetOutput(uint8_t motor, int8_t dir, uint16_t permille);
static void MotorTest_Start(uint8_t motor, int8_t dir, uint16_t permille, uint32_t duration_ms);
static void MotorTest_RunFullA(void);
static void MotorTest_DriveAll(int8_t dir);
static void MotorTest_SquareStart(void);
static void MotorTest_SquareStep(void);
static void MotorTest_SquareApply(uint8_t leg);
static void MotorTest_TurnLeftStart(void);
static void MotorTest_TurnLeftStep(void);
static void MotorTest_TurnRightStart(void);
static void MotorTest_TurnRightStep(void);
static void MotorTest_KeyScan(void);
static void MotorTest_StopOutputs(void);
static void MotorTest_RunSingle(uint8_t motor, int8_t dir);
static void MotorTest_AutoStart(void);
static void MotorTest_AutoCancel(void);
static void MotorTest_AutoStep(void);
static int32_t MotorTest_ReadEncoder(uint8_t motor);
static void MotorTest_ResetEncoders(void);
static uint32_t MotorTest_ReadBatteryMv(void);
static void MotorTest_HandleCommand(char *line);
static uint8_t MotorTest_DipSwitchMode(void);
static void Uart_SendText(const char *text);
static void Uart_SendU32(uint32_t value);
static void Uart_SendS32(int32_t value);
static void Uart_SendMotor(uint8_t motor);
static const char *SkipSpaces(const char *p);
static uint8_t ParseU32(const char **p, uint32_t *value);
static uint8_t ParseFloat(const char **p, float *value);
static uint8_t ParseMotor(const char **p, uint8_t *motor);
static void TrimCommand(char *line);
static uint8_t IsWhitespaceOnly(const char *s, uint8_t len);
static uint8_t IsSingleCommandChar(uint8_t c);

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
  MotorTest_Init();
  Uart_SendText("\r\nWULONG motor test ready\r\n");
  Uart_SendText("Default: STBY low, all PWM=0. Type H for help.\r\n");

  xTaskCreate(ControlTask, "control", 256, NULL, 4, &control_task_handle);
  xTaskCreate(CommsTask, "comms", 256, NULL, 3, &comms_task_handle);
  xTaskCreate(StrategyTask, "strategy", 384, NULL, 2, &strategy_task_handle);

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
  square_test_active = 0U;
  turn_left_active = 0U;
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
}

static void MotorTest_SetOutput(uint8_t motor, int8_t dir, uint16_t permille)
{
  static const uint32_t channels[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
  };
  uint32_t pulse;

  if (motor > 3U || dir == 0) {
    return;
  }
  if (permille > 300U) {
    permille = 300U;
  }
  pulse = ((uint32_t)permille * 16999U) / 1000U;

  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_SET);

  switch (motor) {
  case 0:
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, dir > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(GPIOE, BIN1_Pin, dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, BIN2_Pin, dir > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case 2:
    /*
     * Motor C is mounted mirrored relative to A/B/D.
     * Invert its logical direction so + is the same physical forward direction.
     */
    HAL_GPIO_WritePin(GPIOB, CIN1_Pin, dir > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, CIN2_Pin, dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    break;
  default:
    HAL_GPIO_WritePin(GPIOD, DIN1_Pin, dir > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, DIN2_Pin, dir > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channels[motor], pulse);
}

static void MotorTest_RunFullA(void)
{
  MotorTest_AutoCancel();
  square_test_active = 0U;
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
  square_test_active = 0U;
  turn_left_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  /*
   * Drive all four wheels in the same logical direction.
   * Motor C already has its logical direction inverted for its mirrored mount.
   * Keep the first driving test at 20% duty; use S/E for immediate stop.
   */
  MotorTest_SetOutput(0U, dir, 200U);
  MotorTest_SetOutput(1U, dir, 200U);
  MotorTest_SetOutput(2U, dir, 200U);
  MotorTest_SetOutput(3U, dir, 200U);

  drive_mode_active = 1U;
  active_motor = 0U;
  active_dir = dir;
  active_until_ms = 0U;

  Uart_SendText(dir > 0 ? "DRIVE FORWARD duty=200\r\n"
                        : "DRIVE REVERSE duty=200\r\n");
}

/*
 * 20 cm square path.
 *
 * Distance conversion:
 *   1456 encoder counts = 1 wheel revolution
 *   one revolution = pi * 60 mm = 188.50 mm
 *   => 1 mm = 7.724 counts
 *   200 mm = 1545 counts (approx.)
 *
 * The wheel direction pattern below assumes this logical forward direction:
 *   +vx = forward, -vx = backward, +vy = left, -vy = right.
 * If the first square run moves in the wrong physical direction, only the
 * corresponding sign pattern needs correcting.
 */
#define SQUARE_TARGET_COUNTS  1545
#define SQUARE_DUTY_PERMILLE  200U

static uint8_t  square_leg = 0U;
static uint32_t square_leg_start_ms = 0U;
static const char *square_leg_names[4] = { "FWD", "TURN_L", "BWD", "TURN_R" };

static void MotorTest_SquareStart(void)
{
  if (square_test_active) {
    return;
  }

  MotorTest_AutoCancel();
  turn_left_active = 0U;
  turn_right_active = 0U;
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_ResetEncoders();
  MotorTest_Arm();

  square_leg = 0U;
  square_test_active = 1U;
  square_leg_start_ms = HAL_GetTick();
  MotorTest_SquareApply(0U);
  Uart_SendText("K1 PATH START: FWD TURN_L BWD TURN_R\r\n");
  Uart_SendText("K1 LEG FWD\r\n");
}

static void MotorTest_SquareApply(uint8_t leg)
{
  /*
   * leg 0 = forward, 1 = turn left, 2 = backward, 3 = turn right
   * Lateral strafe was abandoned after the rollers proved unable to
   * generate enough sideways grip.  The square path now uses pivot turns,
   * which rely on longitudinal friction only.
   * Patterns use the confirmed physical wheel map:
   *   A=FL, B=RL, C=RR, D=FR
   * and the mecanum inverse kinematics from docs/03.
   */
  switch (leg) {
  case 0U:
    /* forward */
    MotorTest_SetOutput(WHEEL_FL,  1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RL,  1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RR,  1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_FR,  1, SQUARE_DUTY_PERMILLE);
    break;
  case 1U:
    /* turn left (CCW): A- B- C+ D+ */
    MotorTest_SetOutput(WHEEL_FL, -1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RL, -1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RR,  1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_FR,  1, SQUARE_DUTY_PERMILLE);
    break;
  case 2U:
    /* backward */
    MotorTest_SetOutput(WHEEL_FL, -1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RL, -1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RR, -1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_FR, -1, SQUARE_DUTY_PERMILLE);
    break;
  default:
    /* turn right (CW): A+ B+ C- D- */
    MotorTest_SetOutput(WHEEL_FL,  1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RL,  1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_RR, -1, SQUARE_DUTY_PERMILLE);
    MotorTest_SetOutput(WHEEL_FR, -1, SQUARE_DUTY_PERMILLE);
    break;
  }
}

/*
 * Turn left (counter-clockwise) in place for 1000 ms.
 *
 * Mecanum pivot-turn pattern:
 *   A- B+ C- D+ (logical directions; C has mirrored hardware correction).
 *
 * Note: lateral motion was abandoned because the mecanum rollers could not
 * generate enough sideways grip.  Turn-in-place uses the same wheel torque
 * direction but is no longer called strafe.
 */
#define TURN_LEFT_DUTY_PERMILLE  200U
#define TURN_LEFT_DURATION_MS    1000U

static uint32_t turn_left_until_ms = 0U;

static void MotorTest_TurnLeftStart(void)
{
  if (square_test_active || turn_left_active) {
    return;
  }

  MotorTest_AutoCancel();
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  /* turn left (CCW): A- B- C+ D+ */
  MotorTest_SetOutput(WHEEL_FL, -1, TURN_LEFT_DUTY_PERMILLE);
  MotorTest_SetOutput(WHEEL_RL, -1, TURN_LEFT_DUTY_PERMILLE);
  MotorTest_SetOutput(WHEEL_RR,  1, TURN_LEFT_DUTY_PERMILLE);
  MotorTest_SetOutput(WHEEL_FR,  1, TURN_LEFT_DUTY_PERMILLE);

  turn_left_active = 1U;
  turn_left_until_ms = HAL_GetTick() + TURN_LEFT_DURATION_MS;
  Uart_SendText("TURN LEFT duty=200 ms=1000\r\n");
}

static void MotorTest_TurnLeftStep(void)
{
  if (!turn_left_active) {
    return;
  }

  if ((int32_t)(HAL_GetTick() - turn_left_until_ms) >= 0) {
    turn_left_active = 0U;
    MotorTest_StopOutputs();
    Uart_SendText("TURN LEFT DONE\r\n");
  }
}

/*
 * Turn right (clockwise) in place for 1000 ms.
 * Pattern is the mirror of turn left:
 *   A+ B- C+ D-
 */
#define TURN_RIGHT_DUTY_PERMILLE  200U
#define TURN_RIGHT_DURATION_MS    1000U

static uint32_t turn_right_until_ms = 0U;

static void MotorTest_TurnRightStart(void)
{
  if (square_test_active || turn_left_active || turn_right_active) {
    return;
  }

  MotorTest_AutoCancel();
  drive_mode_active = 0U;
  MotorTest_StopOutputs();
  MotorTest_Arm();

  /* turn right (CW): A+ B+ C- D- */
  MotorTest_SetOutput(WHEEL_FL,  1, TURN_RIGHT_DUTY_PERMILLE);
  MotorTest_SetOutput(WHEEL_RL,  1, TURN_RIGHT_DUTY_PERMILLE);
  MotorTest_SetOutput(WHEEL_RR, -1, TURN_RIGHT_DUTY_PERMILLE);
  MotorTest_SetOutput(WHEEL_FR, -1, TURN_RIGHT_DUTY_PERMILLE);

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


static uint32_t MotorTest_SquareAverageAbsCount(void)
{
  int32_t a = MotorTest_ReadEncoder(0);
  int32_t b = MotorTest_ReadEncoder(1);
  int32_t c = MotorTest_ReadEncoder(2);
  int32_t d = MotorTest_ReadEncoder(3);
  uint64_t sum;

  if (a < 0) { a = -a; }
  if (b < 0) { b = -b; }
  if (c < 0) { c = -c; }
  if (d < 0) { d = -d; }
  sum = (uint64_t)(uint32_t)a + (uint64_t)(uint32_t)b +
        (uint64_t)(uint32_t)c + (uint64_t)(uint32_t)d;
  return (uint32_t)(sum / 4U);
}

static void MotorTest_SquareStep(void)
{
  uint32_t avg_count;
  uint32_t elapsed_ms;

  if (!square_test_active) {
    return;
  }

  elapsed_ms = (uint32_t)(HAL_GetTick() - square_leg_start_ms);
  avg_count = MotorTest_SquareAverageAbsCount();
  if (avg_count >= SQUARE_TARGET_COUNTS || elapsed_ms >= 4000U) {
    if (avg_count < SQUARE_TARGET_COUNTS) {
      square_test_active = 0U;
      MotorTest_StopOutputs();
      Uart_SendText("K1 SQUARE ABORT: leg timeout\r\n");
      return;
    }

    MotorTest_StopOutputs();
    if (++square_leg >= 4U) {
      square_test_active = 0U;
      Uart_SendText("K1 SQUARE DONE\r\n");
      return;
    }

    MotorTest_ResetEncoders();
    MotorTest_Arm();
    square_leg_start_ms = HAL_GetTick();
    Uart_SendText("K1 LEG ");
    Uart_SendText(square_leg_names[square_leg]);
    Uart_SendText("\r\n");
  }

  MotorTest_SquareApply(square_leg);
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
      MotorTest_SquareStart();
    }
  }

  if (raw_k2 != k2_last_raw) {
    k2_last_raw = raw_k2;
    k2_last_change_ms = now;
  } else if ((uint32_t)(now - k2_last_change_ms) >= 30U &&
             raw_k2 != k2_stable) {
    k2_stable = raw_k2;
    if (k2_stable == 0U) {
      MotorTest_TurnLeftStart();
    }
  }

  if (raw_k3 != k3_last_raw) {
    k3_last_raw = raw_k3;
    k3_last_change_ms = now;
  } else if ((uint32_t)(now - k3_last_change_ms) >= 30U &&
             raw_k3 != k3_stable) {
    k3_stable = raw_k3;
    if (k3_stable == 0U) {
      /* Lateral strafe abandoned: K3 intentionally does nothing. */
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
  MotorTest_SetOutput(motor, dir, permille);
}

static void MotorTest_RunSingle(uint8_t motor, int8_t dir)
{
  MotorTest_AutoCancel();
  square_test_active = 0U;
  turn_left_active = 0U;
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

static void MotorTest_AutoStart(void)
{
  MotorTest_AutoCancel();
  square_test_active = 0U;
  turn_left_active = 0U;
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
    MotorTest_SetOutput(motor, dir, 100U);
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
  switch (motor) {
  case 0:
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  case 1:
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  case 2:
    /* Keep the encoder sign aligned with the inverted logical direction. */
    return -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  default:
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim20);
  }
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

static uint8_t IsWhitespaceOnly(const char *s, uint8_t len)
{
  uint8_t i;

  for (i = 0U; i < len; ++i) {
    if (s[i] != ' ' && s[i] != '\t') {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t IsSingleCommandChar(uint8_t c)
{
  switch (c) {
  case 'H': case 'h': case '?':
  case 'S': case 's':
  case 'E': case 'e':
  case 'Z': case 'z':
  case 'V': case 'v':
  case 'P': case 'p':
  case 'F': case 'f':
  case 'N': case 'n':
  case 'R': case 'r':
  case 'W': case 'w':
  case 'X': case 'x':
  case 'Q': case 'q':
  case 'T': case 't':
  case '1': case '2': case '3': case '4':
  case '5': case '6': case '7': case '8':
    return 1U;
  default:
    return 0U;
  }
}

static void Uart_SendText(const char *text)
{
  if (text != NULL) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
  }
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
    Uart_SendText("  N = motor A reverse 10% for 800 ms\r\n");
    Uart_SendText("  1/2=A +/-  3/4=B +/-  5/6=C +/-  7/8=D +/-\r\n");
    Uart_SendText("  K2 = turn LEFT 20% for 1000 ms\r\n");
    Uart_SendText("  R  = turn RIGHT 20% for 1000 ms\r\n");
    Uart_SendText("  K3 = disabled (lateral strafe abandoned)\r\n");
    Uart_SendText("  T = auto test A+ A- B+ B- C+ C- D+ D-\r\n");
    Uart_SendText("  W = drive all wheels forward 20%\r\n");
    Uart_SendText("  X = drive all wheels reverse 20%\r\n");
    Uart_SendText("  Q = print encoder counts (A B C D)\r\n");
    Uart_SendText("  A+ [duty_ms] | A+ <permille> <ms>   (also B/C/D, +/-)\r\n");
    Uart_SendText("  RUN <A-D> <+/-/F/R> <permille> <ms>\r\n");
    Uart_SendText("  ENC | Z(reset encoders) | V(battery mV) | STAT\r\n");
    Uart_SendText("  permille: 1..300 (10=1%), ms: 1..5000\r\n");
    Uart_SendText("  Multi-char commands execute on CR/LF or semicolon.\r\n");
    Uart_SendText("  Single-char H/S/E/Z/V/Q/P/N/T/1-8 execute immediately.\r\n\r\n");
    Uart_SendText(" Calibration console (closed-loop speed control):\r\n");
    Uart_SendText("  CV            toggle console on/off\r\n");
    Uart_SendText("  CS <0-3> <mmps>       set one wheel target speed\r\n");
    Uart_SendText("  CG <0-3> <kp> <ki> <kd>  set speed-loop gains\r\n");
    Uart_SendText("  CH <mm>       set half-diagonal lever arm\r\n");
    Uart_SendText("  CM <val>      set mm per encoder count\r\n");
    Uart_SendText("  CP            print wheel speeds, pose, calibration\r\n");
    Uart_SendText("  CZ            zero pose and encoders\r\n\r\n");
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

  /* ---- Calibration console ------------------------------------------- */

  if (strcmp(line, "CV") == 0) {
    MotorTest_AutoCancel();
    square_test_active = 0U;
    turn_left_active = 0U;
    turn_right_active = 0U;
    drive_mode_active = 0U;
    MotorTest_StopOutputs();
    g_ctrl_console_active = (uint8_t)(g_ctrl_console_active ? 0U : 1U);
    if (g_ctrl_console_active) {
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
    return;
  }

  if (strcmp(line, "CZ") == 0) {
    Wulong_ResetPose(0.0f, 0.0f, 0.0f);
    Uart_SendText("POSE ZEROED\r\n");
    return;
  }

  if (strcmp(line, "S") == 0 || strcmp(line, "STOP") == 0) {
    MotorTest_AutoCancel();
    square_test_active = 0U;
    g_ctrl_console_active = 0U;
    Wulong_ControlDisarm();
    MotorTest_Disarm();
    Uart_SendText("STOPPED + DISARMED\r\n");
    return;
  }

  if (strcmp(line, "E") == 0 || strcmp(line, "ESTOP") == 0) {
    MotorTest_AutoCancel();
    square_test_active = 0U;
    g_ctrl_console_active = 0U;
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
    MotorTest_AutoCancel();
    square_test_active = 0U;
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
    Uart_SendText("\r\n");
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

    /*
     * Calibration console mode: run the closed-loop chassis stack at 1 kHz
     * instead of the bring-up test state machines.
     */
    if (g_ctrl_console_active) {
      if (g_estop_request) {
        g_estop_request = 0;
        g_ctrl_console_active = 0U;
        Wulong_ControlDisarm();
        Uart_SendText("ESTOP (console off)\r\n");
      } else {
        Wulong_ControlTick();
      }
      vTaskDelayUntil(&last_wake, period);
      continue;
    }

    MotorTest_KeyScan();
    (void)MotorTest_DipSwitchMode();

    if (g_estop_request) {
      g_estop_request = 0;
      MotorTest_AutoCancel();
      square_test_active = 0U;
      turn_left_active = 0U;
      turn_right_active = 0U;
          MotorTest_Disarm();
      Uart_SendText("ESTOP\r\n");
    }

    if (g_test_trigger) {
      g_test_trigger = 0;
      if (!square_test_active) {
        MotorTest_Arm();
        MotorTest_Start((uint8_t)g_test_motor,
                        (int8_t)g_test_dir,
                        (uint16_t)g_test_duty_permille,
                        (uint32_t)g_test_duration_ms);
        Uart_SendText("WATCH TEST START\r\n");
      }
    }

    if (!auto_test_active && !drive_mode_active && !square_test_active &&
        !turn_left_active && !turn_right_active &&
        active_dir != 0 &&
        (int32_t)(now - active_until_ms) >= 0) {
      MotorTest_Disarm();
      Uart_SendText("TIMEOUT STOP\r\n");
    }

    MotorTest_AutoStep();
    MotorTest_SquareStep();
    MotorTest_TurnLeftStep();
    MotorTest_TurnRightStep();

    vTaskDelayUntil(&last_wake, period);
  }
}

static void CommsTask(void *argument)
{
  char line[64];
  uint8_t line_len = 0;
  uint8_t rx_byte = 0;

  (void) argument;

  for (;;)
  {
    if (HAL_UART_Receive(&huart1, &rx_byte, 1U, 0U) == HAL_OK) {
      if (rx_byte == '\r' || rx_byte == '\n') {
        if (line_len > 0U) {
          line[line_len] = '\0';
          MotorTest_HandleCommand(line);
          line_len = 0U;
        }
      } else if (rx_byte == ';') {
        if (line_len > 0U) {
          line[line_len] = '\0';
          MotorTest_HandleCommand(line);
          line_len = 0U;
        }
      } else if (line_len == 0U && (rx_byte == ' ' || rx_byte == '\t')) {
        /* Ignore leading whitespace before a command. */
      } else if ((line_len == 0U || IsWhitespaceOnly(line, line_len)) &&
                 IsSingleCommandChar(rx_byte)) {
        line[0] = (char)rx_byte;
        line[1] = '\0';
        MotorTest_HandleCommand(line);
        line_len = 0U;
      } else if (line_len < (sizeof(line) - 1U)) {
        line[line_len++] = (char)rx_byte;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
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
