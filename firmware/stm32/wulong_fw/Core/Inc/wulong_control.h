/**
  ******************************************************************************
  * @file    wulong_control.h
  * @brief   Wulong chassis control, odometry and motion primitives.
  *
  * Hardware baseline (confirmed 2026-10-01):
  *   - 4x MG513X, 1:28, 1456 encoder counts per output-shaft revolution
  *   - phi 60 mm mecanum wheels
  *   - TB6612 D24A, 10 kHz PWM, STBY on PC6
  *   - Lateral strafe is NOT used: roller grip is insufficient.
  *
  * Physical wheel map:
  *   A = front-left, B = rear-left, C = rear-right, D = front-right
  *
  * Supported body motions: forward/backward translation and pivot turns only.
  ******************************************************************************
  */

#ifndef __WULONG_CONTROL_H__
#define __WULONG_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

#define WULONG_CONTROL_DT_MS        1U        /* control tick, 1 kHz */

#define WULONG_ENCODER_COUNTS_REV   1456.0f   /* 13 PPR x 2 x 28 x 4 */
#define WULONG_WHEEL_DIAMETER_MM    60.0f
#define WULONG_WHEEL_RADIUS_MM      30.0f
#define WULONG_WHEEL_CIRC_MM        188.4956f
#define WULONG_MM_PER_COUNT         0.129461f

/*
 * (half wheelbase + half track), in mm.  This is the lever arm used by the
 * pivot-turn term.  MUST be calibrated: command a 1 mrad turn and compare the
 * measured yaw against this value.  Default is a placeholder.
 */
#define WULONG_HALF_DIAG_MM         100.0f    /* TODO: calibrate */

/* Duty limit for normal driving.  300 = 30 % keeps TB6612 well inside its
 * 1.2 A continuous rating for MG513X.  Use 1000 only for bench tests. */
#define WULONG_DUTY_LIMIT           300

/* ------------------------------------------------------------------------- */
/* Types                                                                     */
/* ------------------------------------------------------------------------- */

typedef enum {
  WULONG_FL = 0,   /* A */
  WULONG_RL = 1,   /* B */
  WULONG_RR = 2,   /* C */
  WULONG_FR = 3,   /* D */
  WULONG_WHEEL_COUNT = 4
} Wulong_Wheel;

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_err;
  float out_min;
  float out_max;
} Wulong_Pid;

/* Vehicle pose in the field frame, mm and milliradians. */
typedef struct {
  float x_mm;
  float y_mm;
  float theta_mrad;
} Wulong_Pose;

/* Latest vision result pushed in by the UART layer (single slot, no queue). */
typedef struct {
  volatile uint8_t fresh;
  int16_t x_mm;          /* forward, body frame */
  int16_t y_mm;          /* right, body frame */
  uint8_t color;         /* 0 unknown, 1 red, 2 black */
  uint8_t conf;          /* 0..100 */
  uint32_t timestamp_ms;
} Wulong_VisionTarget;

/* Motion primitive state. */
typedef enum {
  WULONG_MOTION_IDLE = 0,
  WULONG_MOTION_DRIVE_DISTANCE,
  WULONG_MOTION_TURN_ANGLE,
  WULONG_MOTION_DONE,
  WULONG_MOTION_ABORTED
} Wulong_MotionState;

typedef struct {
  Wulong_MotionState state;
  float vx_cmd_mmps;
  float omega_cmd_mradps;
  uint32_t start_ms;
  float start_x_mm;
  float start_y_mm;
  float start_theta_mrad;
  float target_dist_mm;      /* signed */
  float target_angle_mrad;   /* signed */
} Wulong_Motion;

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void   Wulong_ControlInit(void);
void   Wulong_ControlDisarm(void);

/* Direct wheel access, signed permille (-1000..1000), positive = logical
 * forward.  Used by bring-up tests. */
void   Wulong_SetWheelDuty(Wulong_Wheel wheel, int16_t duty_permille);

/* Body-frame velocity command.  vy is not supported (strafe abandoned).
 * vx in mm/s, omega in milliradians/s (CCW positive). */
void   Wulong_SetBodyVelocity(float vx_mmps, float omega_mradps);
void   Wulong_Stop(void);

/* 1 kHz tick: runs the per-wheel speed PI loop and the odometry integrator.
 * Must be called from the control task every WULONG_CONTROL_DT_MS. */
void   Wulong_ControlTick(void);

/* Odometry -----------------------------------------------------------------*/
Wulong_Pose Wulong_GetPose(void);
void        Wulong_ResetPose(float x_mm, float y_mm, float theta_mrad);
int32_t     Wulong_GetEncoder(Wulong_Wheel wheel);
void        Wulong_ResetEncoders(void);
void        Wulong_Relocalize(float x_mm, float y_mm, float theta_mrad);

/* Motion primitives (non-blocking) ------------------------------------------*/
void   Wulong_DriveDistance(float mm, float speed_mmps);
void   Wulong_TurnAngle(float mrad, float omega_mradps);
void   Wulong_MotionAbort(void);
Wulong_MotionState Wulong_MotionUpdate(void);
uint8_t Wulong_MotionIsBusy(void);

/* Helpers -------------------------------------------------------------------*/
void   Wulong_PidInit(Wulong_Pid *pid, float kp, float ki, float kd,
                      float out_min, float out_max);
float  Wulong_PidUpdate(Wulong_Pid *pid, float error, float dt_s);

/* Current body velocity command actually applied by the speed loop. */
void   Wulong_GetBodyVelocity(float *vx_mmps, float *omega_mradps);

/* Calibration support -------------------------------------------------------*/

/* Measured wheel linear speed from the last control tick, mm/s. */
float  Wulong_GetWheelSpeed(Wulong_Wheel wheel);

/* Override a single wheel target without touching the others.  Used by the
 * speed-loop step test.  Call Wulong_Stop() to end the test. */
void   Wulong_SetWheelTarget(Wulong_Wheel wheel, float mmps);

/* Live gain update (speed loop).  kp/ki/kd in duty-per-(mm/s) units. */
void   Wulong_SetWheelGains(Wulong_Wheel wheel, float kp, float ki, float kd);
void   Wulong_GetWheelGains(Wulong_Wheel wheel, float *kp, float *ki, float *kd);

/* Runtime calibration constants, settable from the debug console so that the
 * chassis can be calibrated without a reflash.  Values reset on power cycle;
 * copy the final numbers back into the #defines above when done. */
void   Wulong_SetHalfDiag(float half_diag_mm);
float  Wulong_GetHalfDiag(void);
void   Wulong_SetMmPerCount(float mm_per_count);
float  Wulong_GetMmPerCount(void);

/* Encoder-derived distance travelled by one wheel since the last reset. */
float  Wulong_GetWheelDistance(Wulong_Wheel wheel);

#ifdef __cplusplus
}
#endif

#endif /* __WULONG_CONTROL_H__ */
