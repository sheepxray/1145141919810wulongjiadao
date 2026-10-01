/**
  ******************************************************************************
  * @file    wulong_control.c
  * @brief   Chassis control, odometry and motion primitives.
  *          See wulong_control.h for the hardware baseline.
  ******************************************************************************
  */

#include "wulong_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>

/* ------------------------------------------------------------------------- */
/* Private state                                                             */
/* ------------------------------------------------------------------------- */

/* Per-wheel speed loop.  Gains are a starting point and must be tuned on the
 * real chassis.  Output is signed duty permille. */
static Wulong_Pid s_speed_pid[WULONG_WHEEL_COUNT];

/* Target wheel linear speed (mm/s) produced by the inverse kinematics. */
static float s_wheel_target_mmps[WULONG_WHEEL_COUNT];

/* Previous encoder counts and previous tick time, for delta computation. */
static int32_t  s_enc_prev[WULONG_WHEEL_COUNT];
static uint32_t s_last_tick_ms;

/* Measured wheel speed from the previous tick, mm/s. */
static float s_wheel_speed_mmps[WULONG_WHEEL_COUNT];

/* Runtime calibration constants.  Initialised from the header #defines but
 * adjustable from the debug console (see header comment). */
static float s_half_diag_mm = WULONG_HALF_DIAG_MM;
static float s_mm_per_count = WULONG_MM_PER_COUNT;

/* Odometry pose and per-wheel accumulated distance since last reset. */
static Wulong_Pose s_pose;
static float       s_wheel_dist_mm[WULONG_WHEEL_COUNT];

/* Body velocity command. */
static float s_vx_cmd_mmps;
static float s_omega_cmd_mradps;
static uint8_t s_armed;

/* Active motion primitive. */
static Wulong_Motion s_motion;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static float clampf(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

void Wulong_PidInit(Wulong_Pid *pid, float kp, float ki, float kd,
                    float out_min, float out_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_err = 0.0f;
  pid->out_min = out_min;
  pid->out_max = out_max;
}

float Wulong_PidUpdate(Wulong_Pid *pid, float error, float dt_s)
{
  float deriv;
  float out;

  pid->integral += error * dt_s;

  /* Anti-windup: clamp the integrator into a range implied by the output
   * limit and the integral gain, so a long saturation cannot build up. */
  if (pid->ki > 1e-6f) {
    float ilim = pid->out_max / pid->ki;
    pid->integral = clampf(pid->integral, -ilim, ilim);
  }

  deriv = (error - pid->prev_err) / dt_s;
  pid->prev_err = error;

  out = pid->kp * error + pid->ki * pid->integral + pid->kd * deriv;
  return clampf(out, pid->out_min, pid->out_max);
}

/* ------------------------------------------------------------------------- */
/* Low-level wheel output                                                    */
/* ------------------------------------------------------------------------- */

/*
 * Signed duty -> TB6612 direction pins + TIM1 compare.
 *
 * Motor C is mounted mirrored, so its direction pins are inverted here; this
 * keeps "+" meaning "wheel drives the chassis forward" for every wheel.
 * The encoder for C is negated in Wulong_GetEncoder to stay consistent.
 */
void Wulong_SetWheelDuty(Wulong_Wheel wheel, int16_t duty_permille)
{
  static const uint32_t channels[WULONG_WHEEL_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
  };
  int16_t d = duty_permille;
  uint32_t pulse;
  uint8_t forward;

  if (wheel >= WULONG_WHEEL_COUNT) {
    return;
  }
  d = (int16_t)clampf((float)d, -1000.0f, 1000.0f);
  forward = (d >= 0) ? 1U : 0U;
  if (d < 0) { d = (int16_t)(-d); }

  pulse = ((uint32_t)d * 16999U) / 1000U;

  HAL_GPIO_WritePin(GPIOC, STBY_Pin, GPIO_PIN_SET);

  switch (wheel) {
  case WULONG_FL:
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case WULONG_RL:
    HAL_GPIO_WritePin(GPIOE, BIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, BIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  case WULONG_RR:
    /* Mirrored mount: invert both direction lines. */
    HAL_GPIO_WritePin(GPIOB, CIN1_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, CIN2_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    break;
  default:
    HAL_GPIO_WritePin(GPIOD, DIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, DIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    break;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channels[wheel], pulse);
}

int32_t Wulong_GetEncoder(Wulong_Wheel wheel)
{
  switch (wheel) {
  case WULONG_FL:
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  case WULONG_RL:
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  case WULONG_RR:
    return -(int16_t)__HAL_TIM_GET_COUNTER(&htim4);  /* match inverted pins */
  default:
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim20);
  }
}

void Wulong_ResetEncoders(void)
{
  uint8_t i;

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim20, 0);

  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    s_enc_prev[i] = 0;
    s_wheel_dist_mm[i] = 0.0f;
  }
}

/* ------------------------------------------------------------------------- */
/* Inverse kinematics                                                        */
/* ------------------------------------------------------------------------- */

/*
 * Body velocity -> wheel linear speeds.  Lateral motion is deliberately not
 * supported (see the header comment), so this reduces to:
 *
 *   wheel = vx  +/-  omega * half_diagonal
 *
 * Left wheels (FL, RL) get -omega*d for CCW-positive omega, right wheels
 * (RR, FR) get +omega*d, which drives the two sides in opposite directions.
 */
static void body_to_wheel_speeds(float vx_mmps, float omega_mradps)
{
  float d = s_half_diag_mm;
  float w = omega_mradps * 0.001f;   /* mrad/s -> rad/s */
  float spin = w * d;                /* mm/s contribution */

  s_wheel_target_mmps[WULONG_FL] = vx_mmps - spin;
  s_wheel_target_mmps[WULONG_RL] = vx_mmps - spin;
  s_wheel_target_mmps[WULONG_RR] = vx_mmps + spin;
  s_wheel_target_mmps[WULONG_FR] = vx_mmps + spin;
}

void Wulong_SetBodyVelocity(float vx_mmps, float omega_mradps)
{
  s_vx_cmd_mmps = vx_mmps;
  s_omega_cmd_mradps = omega_mradps;
  body_to_wheel_speeds(vx_mmps, omega_mradps);
}

void Wulong_GetBodyVelocity(float *vx_mmps, float *omega_mradps)
{
  if (vx_mmps != 0) {
    *vx_mmps = s_vx_cmd_mmps;
  }
  if (omega_mradps != 0) {
    *omega_mradps = s_omega_cmd_mradps;
  }
}

/* ------------------------------------------------------------------------- */
/* Calibration support                                                       */
/* ------------------------------------------------------------------------- */

float Wulong_GetWheelSpeed(Wulong_Wheel wheel)
{
  if (wheel >= WULONG_WHEEL_COUNT) {
    return 0.0f;
  }
  return s_wheel_speed_mmps[wheel];
}

float Wulong_GetWheelDistance(Wulong_Wheel wheel)
{
  if (wheel >= WULONG_WHEEL_COUNT) {
    return 0.0f;
  }
  return s_wheel_dist_mm[wheel];
}

void Wulong_SetWheelTarget(Wulong_Wheel wheel, float mmps)
{
  if (wheel >= WULONG_WHEEL_COUNT) {
    return;
  }
  s_wheel_target_mmps[wheel] = mmps;
}

void Wulong_SetWheelGains(Wulong_Wheel wheel, float kp, float ki, float kd)
{
  if (wheel >= WULONG_WHEEL_COUNT) {
    return;
  }
  s_speed_pid[wheel].kp = kp;
  s_speed_pid[wheel].ki = ki;
  s_speed_pid[wheel].kd = kd;
}

void Wulong_GetWheelGains(Wulong_Wheel wheel, float *kp, float *ki, float *kd)
{
  if (wheel >= WULONG_WHEEL_COUNT) {
    return;
  }
  if (kp != 0) { *kp = s_speed_pid[wheel].kp; }
  if (ki != 0) { *ki = s_speed_pid[wheel].ki; }
  if (kd != 0) { *kd = s_speed_pid[wheel].kd; }
}

void Wulong_SetHalfDiag(float half_diag_mm)
{
  if (half_diag_mm > 10.0f && half_diag_mm < 500.0f) {
    s_half_diag_mm = half_diag_mm;
  }
}

float Wulong_GetHalfDiag(void)
{
  return s_half_diag_mm;
}

void Wulong_SetMmPerCount(float mm_per_count)
{
  if (mm_per_count > 0.01f && mm_per_count < 1.0f) {
    s_mm_per_count = mm_per_count;
  }
}

float Wulong_GetMmPerCount(void)
{
  return s_mm_per_count;
}

void Wulong_Stop(void)
{
  uint8_t i;

  s_vx_cmd_mmps = 0.0f;
  s_omega_cmd_mradps = 0.0f;
  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    s_wheel_target_mmps[i] = 0.0f;
    Wulong_SetWheelDuty((Wulong_Wheel)i, 0);
  }
}

void Wulong_ControlDisarm(void)
{
  s_armed = 0U;
  Wulong_Stop();
}

void Wulong_ControlInit(void)
{
  uint8_t i;

  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    Wulong_PidInit(&s_speed_pid[i], 1.6f, 3.0f, 0.0f,
                   (float)-WULONG_DUTY_LIMIT, (float)WULONG_DUTY_LIMIT);
    s_wheel_target_mmps[i] = 0.0f;
    s_enc_prev[i] = Wulong_GetEncoder((Wulong_Wheel)i);
    s_wheel_dist_mm[i] = 0.0f;
  }
  s_last_tick_ms = HAL_GetTick();
  s_pose.x_mm = 0.0f;
  s_pose.y_mm = 0.0f;
  s_pose.theta_mrad = 0.0f;
  s_motion.state = WULONG_MOTION_IDLE;
  s_armed = 1U;
  Wulong_Stop();
}

/* ------------------------------------------------------------------------- */
/* 1 kHz control tick                                                        */
/* ------------------------------------------------------------------------- */

void Wulong_ControlTick(void)
{
  uint32_t now = HAL_GetTick();
  float dt_s = (float)(now - s_last_tick_ms) * 0.001f;
  float dtheta_rad;
  uint8_t i;

  if (dt_s <= 0.0f) {
    return;
  }
  s_last_tick_ms = now;

  /* Per-wheel speed loop.  Encoder delta -> mm/s. */
  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    int32_t cnt = Wulong_GetEncoder((Wulong_Wheel)i);
    int32_t delta = cnt - s_enc_prev[i];
    float dist_mm = (float)delta * s_mm_per_count;
    float actual_mmps = dist_mm / dt_s;
    float duty;

    s_enc_prev[i] = cnt;
    s_wheel_dist_mm[i] += dist_mm;
    s_wheel_speed_mmps[i] = actual_mmps;

    if (!s_armed) {
      continue;
    }

    duty = Wulong_PidUpdate(&s_speed_pid[i],
                            s_wheel_target_mmps[i] - actual_mmps,
                            dt_s);
    Wulong_SetWheelDuty((Wulong_Wheel)i, (int16_t)duty);
  }

  /* Odometry: forward speed is the mean of all four wheels, yaw rate comes
   * from the left/right difference. */
  {
    float v_meas = (s_wheel_dist_mm[WULONG_FL] + s_wheel_dist_mm[WULONG_RL] +
                    s_wheel_dist_mm[WULONG_RR] + s_wheel_dist_mm[WULONG_FR]) * 0.25f;
    static float s_dist_prev;
    float ds = v_meas - s_dist_prev;
    float dl = (s_wheel_dist_mm[WULONG_FL] + s_wheel_dist_mm[WULONG_RL]) * 0.5f;
    float dr = (s_wheel_dist_mm[WULONG_RR] + s_wheel_dist_mm[WULONG_FR]) * 0.5f;
    static float s_dl_prev, s_dr_prev;
    float ddl = dl - s_dl_prev;
    float ddr = dr - s_dr_prev;
    float theta_rad = s_pose.theta_mrad * 0.001f;

    s_dist_prev = v_meas;
    s_dl_prev = dl;
    s_dr_prev = dr;

    s_pose.x_mm += ds * cosf(theta_rad);
    s_pose.y_mm += ds * sinf(theta_rad);

    if (s_half_diag_mm > 1.0f) {
      dtheta_rad = (ddr - ddl) / (2.0f * s_half_diag_mm);
      s_pose.theta_mrad += dtheta_rad * 1000.0f;
    }
  }
}

Wulong_Pose Wulong_GetPose(void)
{
  return s_pose;
}

void Wulong_ResetPose(float x_mm, float y_mm, float theta_mrad)
{
  s_pose.x_mm = x_mm;
  s_pose.y_mm = y_mm;
  s_pose.theta_mrad = theta_mrad;
  Wulong_ResetEncoders();
}

void Wulong_Relocalize(float x_mm, float y_mm, float theta_mrad)
{
  s_pose.x_mm = x_mm;
  s_pose.y_mm = y_mm;
  s_pose.theta_mrad = theta_mrad;
}

/* ------------------------------------------------------------------------- */
/* Motion primitives                                                         */
/* ------------------------------------------------------------------------- */

void Wulong_DriveDistance(float mm, float speed_mmps)
{
  s_motion.state = WULONG_MOTION_DRIVE_DISTANCE;
  s_motion.target_dist_mm = mm;
  s_motion.vx_cmd_mmps = (mm >= 0.0f) ? fabsf(speed_mmps) : -fabsf(speed_mmps);
  s_motion.omega_cmd_mradps = 0.0f;
  s_motion.start_x_mm = s_pose.x_mm;
  s_motion.start_y_mm = s_pose.y_mm;
  s_motion.start_ms = HAL_GetTick();
}

void Wulong_TurnAngle(float mrad, float omega_mradps)
{
  s_motion.state = WULONG_MOTION_TURN_ANGLE;
  s_motion.target_angle_mrad = mrad;
  s_motion.vx_cmd_mmps = 0.0f;
  s_motion.omega_cmd_mradps = (mrad >= 0.0f)
                                ? fabsf(omega_mradps)
                                : -fabsf(omega_mradps);
  s_motion.start_theta_mrad = s_pose.theta_mrad;
  s_motion.start_ms = HAL_GetTick();
}

void Wulong_MotionAbort(void)
{
  s_motion.state = WULONG_MOTION_ABORTED;
  Wulong_Stop();
}

uint8_t Wulong_MotionIsBusy(void)
{
  return (s_motion.state == WULONG_MOTION_DRIVE_DISTANCE ||
          s_motion.state == WULONG_MOTION_TURN_ANGLE) ? 1U : 0U;
}

Wulong_MotionState Wulong_MotionUpdate(void)
{
  float travelled;
  float turned;
  const float dist_tol = 3.0f;     /* mm */
  const float angle_tol = 15.0f;   /* mrad = 0.86 deg */
  const uint32_t timeout_ms = 6000U;

  switch (s_motion.state) {
  case WULONG_MOTION_DRIVE_DISTANCE:
    travelled = sqrtf((s_pose.x_mm - s_motion.start_x_mm) *
                      (s_pose.x_mm - s_motion.start_x_mm) +
                      (s_pose.y_mm - s_motion.start_y_mm) *
                      (s_pose.y_mm - s_motion.start_y_mm));

    if ((HAL_GetTick() - s_motion.start_ms) > timeout_ms) {
      s_motion.state = WULONG_MOTION_ABORTED;
      Wulong_Stop();
      break;
    }
    if (travelled >= fabsf(s_motion.target_dist_mm) - dist_tol) {
      s_motion.state = WULONG_MOTION_DONE;
      Wulong_Stop();
      break;
    }
    Wulong_SetBodyVelocity(s_motion.vx_cmd_mmps, 0.0f);
    break;

  case WULONG_MOTION_TURN_ANGLE:
    turned = s_pose.theta_mrad - s_motion.start_theta_mrad;

    if ((HAL_GetTick() - s_motion.start_ms) > timeout_ms) {
      s_motion.state = WULONG_MOTION_ABORTED;
      Wulong_Stop();
      break;
    }
    if (fabsf(turned) >= fabsf(s_motion.target_angle_mrad) - angle_tol) {
      s_motion.state = WULONG_MOTION_DONE;
      Wulong_Stop();
      break;
    }
    Wulong_SetBodyVelocity(0.0f, s_motion.omega_cmd_mradps);
    break;

  default:
    break;
  }

  return s_motion.state;
}
