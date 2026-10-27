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
#include "imu.h"
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

/* Low-passed wheel speed fed to the PID and to the CP display.  See
 * WULONG_SPEED_FILTER_TAU_S for why the raw 1 ms value is unusable. */
static float s_speed_filt[WULONG_WHEEL_COUNT];
static float s_speed_hist[WULONG_WHEEL_COUNT][3];
static uint8_t s_speed_hist_index[WULONG_WHEEL_COUNT];

/* Encoder samples discarded by the plausibility limit (see the header). */
static uint32_t s_enc_reject_count;

/*
 * Per-wheel polarity flip.
 *
 * A wheel mounted the other way round needs BOTH its direction lines inverted
 * AND its encoder sign negated.  Do only one of the two and the encoder still
 * reads positive while the wheel spins backwards - which is exactly the trap
 * that cost us a day on wheels C and B: the per-channel sign test cannot see
 * it, only a physical test can.
 *
 * Defaults below match the assembled chassis as of 2026-10-17.  The bring-up
 * console can change them at runtime with FLIP <wheel> [0|1] while watching the
 * robot drive.
 */
static uint8_t s_flip[WULONG_WHEEL_COUNT] = { 0U, 1U, 1U, 0U };

/* Runtime calibration constants.  Initialised from the header #defines but
 * adjustable from the debug console (see header comment). */
static float s_half_diag_mm = WULONG_HALF_DIAG_MM;
static float s_mm_per_count = WULONG_MM_PER_COUNT;
static float s_lateral_gain = WULONG_MECANUM_LATERAL_GAIN;

/* Odometry pose and per-wheel accumulated distance since last reset. */
static Wulong_Pose s_pose;
static float       s_wheel_dist_mm[WULONG_WHEEL_COUNT];
static float       s_vx_meas_mmps;
static float       s_vy_meas_mmps;

/* Previous odometry accumulators.  These must be reset together with the wheel
 * distances, otherwise a pose reset would produce one large phantom step. */
static float s_dist_prev;
static float s_dl_prev;
static float s_dr_prev;
static float s_vy_prev;

/* Body velocity command. */
static float s_vx_cmd_mmps;
static float s_vy_cmd_mmps;
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

static float median3f(float a, float b, float c)
{
  float lo = (a < b) ? a : b;
  float hi = (a > b) ? a : b;

  if (c < lo) {
    return lo;
  }
  if (c > hi) {
    return hi;
  }
  return c;
}

static float median4f(float a, float b, float c, float d)
{
  float v[4] = { a, b, c, d };
  uint8_t i;
  uint8_t j;

  for (i = 0U; i < 3U; ++i) {
    for (j = (uint8_t)(i + 1U); j < 4U; ++j) {
      if (v[j] < v[i]) {
        float temp = v[i];
        v[i] = v[j];
        v[j] = temp;
      }
    }
  }
  return (v[1] + v[2]) * 0.5f;
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
  pid->last_out = 0.0f;
}

float Wulong_PidUpdate(Wulong_Pid *pid, float error, float dt_s)
{
  return Wulong_PidUpdateWithFeedforward(pid, error, dt_s, 0.0f);
}

float Wulong_PidUpdateWithFeedforward(Wulong_Pid *pid, float error,
                                      float dt_s, float feedforward)
{
  float deriv;
  float candidate_integral;
  float p_term;
  float unclamped;
  float out;

  if (dt_s <= 0.0f) {
    return pid->last_out;
  }

  deriv = (error - pid->prev_err) / dt_s;
  p_term = pid->kp * error;
  candidate_integral = pid->integral + error * dt_s;

  /*
   * Conditional integration: accept an integral update only when it does not
   * push an already saturated output farther into the rail.  This is the
   * discrete equivalent of back-calculation anti-windup used by common motor
   * controllers, without needing a second tuning gain.
   */
  unclamped = feedforward + p_term +
              pid->ki * candidate_integral + pid->kd * deriv;
  if (((unclamped > pid->out_max) && (error > 0.0f)) ||
      ((unclamped < pid->out_min) && (error < 0.0f))) {
    candidate_integral = pid->integral;
  } else {
    pid->integral = candidate_integral;
  }

  unclamped = feedforward + p_term +
              pid->ki * pid->integral + pid->kd * deriv;
  out = clampf(unclamped, pid->out_min, pid->out_max);
  pid->prev_err = error;
  pid->last_out = out;
  return out;
}

/* ------------------------------------------------------------------------- */
/* Low-level wheel output                                                    */
/* ------------------------------------------------------------------------- */

/*
 * Signed duty -> TB6612 direction pins + TIM1 compare.
 *
 * Physical harness mapping (re-checked 2026-10-17 by driving one wheel at a
 * time and watching which corner turns):
 *   logical C (RR wheel) is fed by D24A socket C -> TIM1_CH3 + CIN1/CIN2
 *   logical D (FR wheel) is fed by D24A socket D -> TIM1_CH4 + DIN1/DIN2
 * An earlier revision swapped these two because a sign test suggested the
 * plugs were crossed; the direct wheel-position check showed that was wrong.
 *
 * The RR wheel is mounted mirrored, so its direction lines are inverted;
 * the FR wheel is driven straight.  The matching encoder signs are applied in
 * Wulong_GetEncoder().
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
  if (s_flip[wheel]) {
    forward = (uint8_t)(forward ? 0U : 1U);
  }

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
    HAL_GPIO_WritePin(GPIOB, CIN1_Pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, CIN2_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
  int32_t value;

  switch (wheel) {
  case WULONG_FL:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    break;
  case WULONG_RL:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    break;
  case WULONG_RR:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    break;
  case WULONG_FR:
    value = (int16_t)__HAL_TIM_GET_COUNTER(&htim20);
    break;
  default:
    return 0;
  }

  /* A flipped wheel also has its direction lines inverted, so negate the
   * feedback to keep "positive = this wheel drives the chassis forward". */
  return s_flip[wheel] ? -value : value;
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
    s_speed_filt[i] = 0.0f;
    s_speed_hist[i][0] = 0.0f;
    s_speed_hist[i][1] = 0.0f;
    s_speed_hist[i][2] = 0.0f;
    s_speed_hist_index[i] = 0U;
  }
  s_dist_prev = 0.0f;
  s_dl_prev = 0.0f;
  s_dr_prev = 0.0f;
  s_vy_prev = 0.0f;
  s_vx_meas_mmps = 0.0f;
  s_vy_meas_mmps = 0.0f;
}

/* ------------------------------------------------------------------------- */
/* Inverse kinematics                                                        */
/* ------------------------------------------------------------------------- */

/*
 * Body velocity -> wheel linear speeds for the installed O-layout mecanum
 * wheels.  The local convention is vx forward, vy left:
 *
 *   FL = vx - vy + spin
 *   RL = vx + vy + spin
 *   RR = vx - vy - spin
 *   FR = vx + vy - spin
 *
 * The 2026-10-20 K3 trace measured a +omega command producing negative IMU
 * yaw, so the physical pivot mapping is reversed relative to the original
 * convention.  To keep CCW positive end-to-end, left wheels now get
 * +omega*d and right wheels get -omega*d.
 */
static void body_to_wheel_speeds(float vx_mmps, float vy_mmps,
                                 float omega_mradps)
{
  float d = s_half_diag_mm;
  float w = omega_mradps * 0.001f;   /* mrad/s -> rad/s */
  float spin = w * d;                /* mm/s contribution */

  {
    float wheel_vy = vy_mmps / s_lateral_gain;

    s_wheel_target_mmps[WULONG_FL] = vx_mmps - wheel_vy + spin;
    s_wheel_target_mmps[WULONG_RL] = vx_mmps + wheel_vy + spin;
    s_wheel_target_mmps[WULONG_RR] = vx_mmps - wheel_vy - spin;
    s_wheel_target_mmps[WULONG_FR] = vx_mmps + wheel_vy - spin;
  }
}

void Wulong_SetBodyVelocityXY(float vx_mmps, float vy_mmps,
                              float omega_mradps)
{
  s_vx_cmd_mmps = vx_mmps;
  s_vy_cmd_mmps = vy_mmps;
  s_omega_cmd_mradps = omega_mradps;
  body_to_wheel_speeds(vx_mmps, vy_mmps, omega_mradps);
}

void Wulong_SetBodyVelocity(float vx_mmps, float omega_mradps)
{
  Wulong_SetBodyVelocityXY(vx_mmps, 0.0f, omega_mradps);
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

void Wulong_GetBodyVelocity3(float *vx_mmps, float *vy_mmps,
                             float *omega_mradps)
{
  if (vx_mmps != 0) {
    *vx_mmps = s_vx_cmd_mmps;
  }
  if (vy_mmps != 0) {
    *vy_mmps = s_vy_cmd_mmps;
  }
  if (omega_mradps != 0) {
    *omega_mradps = s_omega_cmd_mradps;
  }
}

void Wulong_GetMeasuredTranslationVelocity(float *vx_mmps, float *vy_mmps)
{
  if (vx_mmps != 0) {
    *vx_mmps = s_vx_meas_mmps;
  }
  if (vy_mmps != 0) {
    *vy_mmps = s_vy_meas_mmps;
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

float Wulong_GetWheelDuty(Wulong_Wheel wheel)
{
  if (wheel >= WULONG_WHEEL_COUNT) {
    return 0.0f;
  }
  return s_speed_pid[wheel].last_out;
}

uint32_t Wulong_GetEncoderRejectCount(void)
{
  return s_enc_reject_count;
}

void Wulong_SetWheelFlip(Wulong_Wheel wheel, uint8_t on)
{
  if (wheel < WULONG_WHEEL_COUNT) {
    s_flip[wheel] = (uint8_t)(on ? 1U : 0U);
  }
}

uint8_t Wulong_GetWheelFlip(Wulong_Wheel wheel)
{
  return (wheel < WULONG_WHEEL_COUNT) ? s_flip[wheel] : 0U;
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
  if ((s_wheel_target_mmps[wheel] * mmps) < 0.0f) {
    s_speed_pid[wheel].integral = 0.0f;
    s_speed_pid[wheel].prev_err = 0.0f;
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

void Wulong_SetLateralGain(float gain)
{
  if (gain > 0.3f && gain < 1.2f) {
    s_lateral_gain = gain;
  }
}

float Wulong_GetLateralGain(void)
{
  return s_lateral_gain;
}

void Wulong_Stop(void)
{
  uint8_t i;

  s_vx_cmd_mmps = 0.0f;
  s_vy_cmd_mmps = 0.0f;
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

void Wulong_ControlArm(void)
{
  s_armed = 1U;
}

void Wulong_ControlInit(void)
{
  uint8_t i;

  for (i = 0U; i < WULONG_WHEEL_COUNT; ++i) {
    /*
     * Gains tuned on the bench (wheels free) on 2026-10-17, after the speed
     * feedback was low-passed.  With the raw 1 ms feedback these values made
     * the loop dither (see WULONG_SPEED_FILTER_TAU_S); with the filter in place
     * they give a 0.6 s rise to 90 % and four-wheel agreement within 0.4 % at
     * 100-200 mm/s.  kd stays 0: the feedback is already averaged and a
     * derivative term would only amplify the residual ripple.
     */
    Wulong_PidInit(&s_speed_pid[i], 5.0f, 15.0f, 0.0f,
                   (float)-WULONG_DUTY_LIMIT, (float)WULONG_DUTY_LIMIT);
    s_wheel_target_mmps[i] = 0.0f;
    s_enc_prev[i] = Wulong_GetEncoder((Wulong_Wheel)i);
    s_wheel_dist_mm[i] = 0.0f;
    s_speed_filt[i] = 0.0f;
    s_speed_hist[i][0] = 0.0f;
    s_speed_hist[i][1] = 0.0f;
    s_speed_hist[i][2] = 0.0f;
    s_speed_hist_index[i] = 0U;
  }
  s_dist_prev = 0.0f;
  s_dl_prev = 0.0f;
  s_dr_prev = 0.0f;
  s_vy_prev = 0.0f;
  s_vx_meas_mmps = 0.0f;
  s_vy_meas_mmps = 0.0f;
  s_last_tick_ms = HAL_GetTick();
  s_pose.x_mm = 0.0f;
  s_pose.y_mm = 0.0f;
  s_pose.theta_mrad = 0.0f;
  if (Imu_IsReady()) {
    Imu_ResetYaw(0.0f);
    Imu_StartBiasCalibration(2000U);
  }
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
    float target_mmps = s_wheel_target_mmps[i];
    float feedforward = 0.0f;
    float duty;

    s_enc_prev[i] = cnt;

    if (actual_mmps > WULONG_MAX_WHEEL_MMPS ||
        actual_mmps < -WULONG_MAX_WHEEL_MMPS) {
      /*
       * Physically impossible rate: this encoder line is being flooded by
       * switching noise from another motor channel (see
       * docs/电机Bring-up测试.md).  Drop the sample - keep the previous speed
       * estimate and contribute no distance - so the speed loop cannot chase
       * the noise up to the duty limit and the odometry cannot integrate
       * garbage.
       */
      ++s_enc_reject_count;
    } else {
      /*
       * Average the 1 ms sample over ~WULONG_SPEED_FILTER_TAU_S before it
       * reaches the PID or the CP display; the raw value only has two or three
       * distinct levels at these speeds.
       */
      float median;
      float tau_s = (i == (uint8_t)WULONG_RR)
                        ? WULONG_SPEED_FILTER_TAU_RR_S
                        : WULONG_SPEED_FILTER_TAU_S;
      float alpha = dt_s / (dt_s + tau_s);
      uint8_t hist_index = s_speed_hist_index[i];

      s_speed_hist[i][hist_index] = actual_mmps;
      hist_index = (uint8_t)((hist_index + 1U) % 3U);
      s_speed_hist_index[i] = hist_index;
      median = median3f(s_speed_hist[i][0], s_speed_hist[i][1],
                        s_speed_hist[i][2]);

      s_wheel_dist_mm[i] += dist_mm;
      s_speed_filt[i] += (median - s_speed_filt[i]) * alpha;
      s_wheel_speed_mmps[i] = s_speed_filt[i];
    }

    if (!s_armed) {
      continue;
    }

    if (fabsf(target_mmps) > 0.5f) {
      feedforward = target_mmps * WULONG_SPEED_FF_PER_MMPS;
      feedforward += (target_mmps > 0.0f)
                       ? WULONG_SPEED_STICTION_PERMILLE
                       : -WULONG_SPEED_STICTION_PERMILLE;
    }

    duty = Wulong_PidUpdateWithFeedforward(
        &s_speed_pid[i], target_mmps - s_wheel_speed_mmps[i],
        dt_s, feedforward);
    Wulong_SetWheelDuty((Wulong_Wheel)i, (int16_t)duty);
  }

  /* Odometry: use a median of four pair averages so one noisy encoder (the
   * known C-channel issue) cannot dominate translational position.  Yaw still
   * comes from the gyro when calibrated. */
  {
    float front = (s_wheel_dist_mm[WULONG_FL] +
                    s_wheel_dist_mm[WULONG_FR]) * 0.5f;
    float rear = (s_wheel_dist_mm[WULONG_RL] +
                   s_wheel_dist_mm[WULONG_RR]) * 0.5f;
    float left = (s_wheel_dist_mm[WULONG_FL] +
                  s_wheel_dist_mm[WULONG_RL]) * 0.5f;
    float right = (s_wheel_dist_mm[WULONG_RR] +
                   s_wheel_dist_mm[WULONG_FR]) * 0.5f;
    float v_meas = median4f(front, rear, left, right);
    float vy_meas =
        (s_wheel_dist_mm[WULONG_FR] + s_wheel_dist_mm[WULONG_RL] -
         s_wheel_dist_mm[WULONG_FL] - s_wheel_dist_mm[WULONG_RR]) * 0.25f;
    float ds = v_meas - s_dist_prev;
    float dy = vy_meas - s_vy_prev;
    float dl = (s_wheel_dist_mm[WULONG_FL] + s_wheel_dist_mm[WULONG_RL]) * 0.5f;
    float dr = (s_wheel_dist_mm[WULONG_RR] + s_wheel_dist_mm[WULONG_FR]) * 0.5f;
    float ddl = dl - s_dl_prev;
    float ddr = dr - s_dr_prev;
    float theta_rad = s_pose.theta_mrad * 0.001f;
    float speed_front = (s_wheel_speed_mmps[WULONG_FL] +
                         s_wheel_speed_mmps[WULONG_FR]) * 0.5f;
    float speed_rear = (s_wheel_speed_mmps[WULONG_RL] +
                        s_wheel_speed_mmps[WULONG_RR]) * 0.5f;
    float speed_left = (s_wheel_speed_mmps[WULONG_FL] +
                        s_wheel_speed_mmps[WULONG_RL]) * 0.5f;
    float speed_right = (s_wheel_speed_mmps[WULONG_RR] +
                         s_wheel_speed_mmps[WULONG_FR]) * 0.5f;

    s_vx_meas_mmps = median4f(speed_front, speed_rear, speed_left, speed_right);
    s_vy_meas_mmps = s_lateral_gain *
        (s_wheel_speed_mmps[WULONG_FR] + s_wheel_speed_mmps[WULONG_RL] -
         s_wheel_speed_mmps[WULONG_FL] - s_wheel_speed_mmps[WULONG_RR]) *
        0.25f;

    s_dist_prev = v_meas;
    s_vy_prev = vy_meas;
    s_dl_prev = dl;
    s_dr_prev = dr;

    /* One bad C-channel sample must not create a large lateral position jump. */
    dy = clampf(dy, -0.20f, 0.20f);

    dy *= s_lateral_gain;
    s_pose.x_mm += ds * cosf(theta_rad) - dy * sinf(theta_rad);
    s_pose.y_mm += ds * sinf(theta_rad) + dy * cosf(theta_rad);

    if (Imu_IsReady() && Imu_CalibrationDone()) {
      s_pose.theta_mrad = Imu_GetYawMrad();
    } else if (!Imu_IsReady() && s_half_diag_mm > 1.0f) {
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
  /* Keep the gyro integrator consistent with the reset pose, otherwise the
   * next tick would immediately overwrite theta with the old integrated yaw. */
  if (Imu_IsReady()) {
    Imu_ResetYaw(theta_mrad);
  }
  Wulong_ResetEncoders();
}

void Wulong_Relocalize(float x_mm, float y_mm, float theta_mrad)
{
  s_pose.x_mm = x_mm;
  s_pose.y_mm = y_mm;
  s_pose.theta_mrad = theta_mrad;
  if (Imu_IsReady()) {
    Imu_ResetYaw(theta_mrad);
  }
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
