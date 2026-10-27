/**
  ******************************************************************************
  * @file    imu.c
  * @brief   ICM-20602 driver over SPI2 + gyro-Z yaw integration.
  *
  * Wiring (see hardware/electrical/接口定义.md):
  *   SCL -> PB13 (SPI2_SCK), SDA -> PB15 (SPI2_MOSI),
  *   SDO -> PB14 (SPI2_MISO), CS -> PB12, INT -> PB9 (reserved).
  *
  * SPI2 is configured as Mode 3, MSB first, 8-bit.  This driver uses polling
  * from the 1 kHz control task, so the data-ready interrupt is left disabled.
  ******************************************************************************
  */

#include "imu.h"
#include "main.h"
#include "spi.h"
#include <math.h>

/* ------------------------------------------------------------------------- */
/* ICM-20602 registers                                                       */
/* ------------------------------------------------------------------------- */

#define ICM20602_WHO_AM_I        0x75U
#define ICM20602_PWR_MGMT_1      0x6BU
#define ICM20602_PWR_MGMT_2      0x6CU
#define ICM20602_CONFIG          0x1AU
#define ICM20602_SMPLRT_DIV      0x19U
#define ICM20602_GYRO_CONFIG     0x1BU
#define ICM20602_ACCEL_CONFIG    0x1CU
#define ICM20602_ACCEL_CONFIG_2  0x1DU
#define ICM20602_INT_PIN_CFG     0x37U
#define ICM20602_INT_ENABLE      0x38U
#define ICM20602_ACCEL_XOUT_H    0x3BU

#define ICM20602_SPI_READ        0x80U
#define ICM20602_WHO_AM_I_VALUE  0x12U

/* +/-500 dps -> 65.5 LSB/(deg/s). */
#define ICM20602_GYRO_LSB_PER_DPS 65.5f
/* +/-4 g -> 8192 LSB/g. */
#define ICM20602_ACC_LSB_PER_G    8192.0f
/* ICM-20602 temperature sensitivity and room-temperature offset. */
#define ICM20602_TEMP_LSB_PER_C   326.8f
#define ICM20602_TEMP_OFFSET_C    25.0f

/* Sign so that CCW chassis rotation gives a positive yaw rate. */
#define IMU_YAW_SIGN             (1.0f)

#define IMU_SPI_TIMEOUT_MS       10U

/*
 * Stationary gate.  A calibrated MEMS gyro still has a small, slowly varying
 * zero-rate output.  Inside the entry threshold it is treated as zero, with
 * hysteresis so measurement noise cannot chatter the integrator.  The bias is
 * also tracked very slowly while stationary to follow temperature drift.
 */
#define IMU_STATIC_XY_DPS        0.50f
#define IMU_STATIC_Z_DPS         0.80f
#define IMU_STATIC_ACC_G         0.05f
#define IMU_STATIC_CONFIRM_MS    300U
#define IMU_BIAS_TRACK_TAU_S     10.0f
#define IMU_CALIB_MAX_SPREAD_DPS 5.0f
#define IMU_GYRO_SATURATION_RAW  32000
#define IMU_YAW_SCALE            1.0f

/* ------------------------------------------------------------------------- */
/* Private state                                                             */
/* ------------------------------------------------------------------------- */

static uint8_t  s_ready;
static uint8_t  s_bias_valid;
static uint8_t  s_calibrating;
static volatile uint8_t  s_cal_request;
static volatile uint16_t s_cal_request_samples;
static uint16_t s_cal_target;
static uint16_t s_cal_count;
static float    s_bias_raw;
static float    s_bias_x_raw;
static float    s_bias_y_raw;
static float    s_cal_sum_x;
static float    s_cal_sum_y;
static float    s_cal_sum_z;
static float    s_cal_sum_sq;
static float    s_cal_acc_mag_sum;
static float    s_cal_min_dps[3];
static float    s_cal_max_dps[3];
static float    s_bias_std_raw;
static float    s_calibration_spread_dps;
static float    s_acc_mag_reference;
static uint8_t  s_stationary;
static float    s_static_ms;
static float    s_yaw_mrad;
static float    s_gyro_z_prev_dps;
static float    s_roll_deg;
static float    s_pitch_deg;
static uint32_t s_gyro_sat_count;
static float    s_gyro_dps[3];
static int16_t  s_acc_raw[3];
static int16_t  s_temp_raw;
static int16_t  s_gyro_raw[3];

/* ------------------------------------------------------------------------- */
/* SPI helpers                                                               */
/* ------------------------------------------------------------------------- */

static uint8_t imu_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];

  tx[0] = reg;
  tx[1] = value;
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi2, tx, 2U, IMU_SPI_TIMEOUT_MS) != HAL_OK) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return 0U;
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return 1U;
}

static uint8_t imu_read_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t cmd = reg | ICM20602_SPI_READ;
  uint8_t ok;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  ok = (HAL_SPI_Transmit(&hspi2, &cmd, 1U, IMU_SPI_TIMEOUT_MS) == HAL_OK) &&
       (HAL_SPI_Receive(&hspi2, data, len, IMU_SPI_TIMEOUT_MS) == HAL_OK);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return ok;
}

static uint8_t imu_read_reg(uint8_t reg, uint8_t *value)
{
  return imu_read_regs(reg, value, 1U);
}

static uint8_t imu_write_reg_verify(uint8_t reg, uint8_t value)
{
  uint8_t readback = 0U;

  if (!imu_write_reg(reg, value)) {
    return 0U;
  }
  if (!imu_read_reg(reg, &readback)) {
    return 0U;
  }
  return (uint8_t)(readback == value);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

uint8_t Imu_Init(void)
{
  uint8_t who = 0U;
  uint8_t value = 0U;
  uint8_t reset_wait;

  s_ready = 0U;
  s_bias_valid = 0U;
  s_calibrating = 0U;
  s_cal_request = 0U;
  s_cal_request_samples = 0U;
  s_cal_target = 0U;
  s_cal_count = 0U;
  s_bias_raw = 0.0f;
  s_bias_x_raw = 0.0f;
  s_bias_y_raw = 0.0f;
  s_cal_sum_x = 0.0f;
  s_cal_sum_y = 0.0f;
  s_cal_sum_z = 0.0f;
  s_cal_sum_sq = 0.0f;
  s_cal_acc_mag_sum = 0.0f;
  s_cal_min_dps[0] = 1.0e9f;
  s_cal_min_dps[1] = 1.0e9f;
  s_cal_min_dps[2] = 1.0e9f;
  s_cal_max_dps[0] = -1.0e9f;
  s_cal_max_dps[1] = -1.0e9f;
  s_cal_max_dps[2] = -1.0e9f;
  s_bias_std_raw = 0.0f;
  s_calibration_spread_dps = 0.0f;
  s_acc_mag_reference = 1.0f;
  s_stationary = 0U;
  s_static_ms = 0.0f;
  s_yaw_mrad = 0.0f;
  s_gyro_z_prev_dps = 0.0f;
  s_roll_deg = 0.0f;
  s_pitch_deg = 0.0f;
  s_gyro_sat_count = 0U;
  s_gyro_dps[0] = 0.0f;
  s_gyro_dps[1] = 0.0f;
  s_gyro_dps[2] = 0.0f;
  s_acc_raw[0] = 0;
  s_acc_raw[1] = 0;
  s_acc_raw[2] = 0;
  s_temp_raw = 0;
  s_gyro_raw[0] = 0;
  s_gyro_raw[1] = 0;
  s_gyro_raw[2] = 0;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);

  if (!imu_read_reg(ICM20602_WHO_AM_I, &who) || who != ICM20602_WHO_AM_I_VALUE) {
    return 0U;
  }

  /*
   * Reference sequence: issue the device reset, then wait until the reset bit
   * actually clears before applying the clock and filter configuration.
   */
  if (!imu_write_reg(ICM20602_PWR_MGMT_1, 0x80U)) {
    return 0U;
  }
  HAL_Delay(5U);
  for (reset_wait = 0U; reset_wait < 20U; ++reset_wait) {
    if (!imu_read_reg(ICM20602_PWR_MGMT_1, &value)) {
      return 0U;
    }
    if ((value & 0x80U) == 0U) {
      break;
    }
    HAL_Delay(1U);
  }
  if ((value & 0x80U) != 0U) {
    return 0U;
  }

  if (!imu_write_reg_verify(ICM20602_PWR_MGMT_1, 0x01U)) {
    return 0U;
  }
  HAL_Delay(10U);
  if (!imu_write_reg_verify(ICM20602_PWR_MGMT_2, 0x00U) ||
      !imu_write_reg_verify(ICM20602_CONFIG, 0x02U) ||
      !imu_write_reg_verify(ICM20602_SMPLRT_DIV, 0x00U) ||
      !imu_write_reg_verify(ICM20602_GYRO_CONFIG, 0x08U) ||
      !imu_write_reg_verify(ICM20602_ACCEL_CONFIG, 0x08U) ||
      !imu_write_reg_verify(ICM20602_ACCEL_CONFIG_2, 0x03U) ||
      !imu_write_reg_verify(ICM20602_INT_PIN_CFG, 0x80U) ||
      !imu_write_reg_verify(ICM20602_INT_ENABLE, 0x00U)) {
    return 0U;
  }

  s_ready = 1U;
  return 1U;
}

uint8_t Imu_IsReady(void)
{
  return s_ready;
}

void Imu_StartBiasCalibration(uint16_t samples)
{
  if (samples == 0U) {
    samples = 1U;
  }
  /* Only post a request; the 1 kHz control task owns the actual state so that
   * a debug-console command can never race the sampling loop. */
  s_cal_request_samples = samples;
  s_cal_request = 1U;
}

uint8_t Imu_CalibrationDone(void)
{
  return (uint8_t)(s_ready && s_bias_valid && !s_calibrating);
}

uint8_t Imu_IsCalibrating(void)
{
  return s_calibrating;
}

void Imu_Update(float dt_s)
{
  uint8_t raw[14];
  float corrected_dps[3];
  float acc_mag;
  uint8_t still_now;
  uint8_t i;

  if (!s_ready) {
    return;
  }

  if (s_cal_request) {
    s_cal_request = 0U;
    s_cal_target = s_cal_request_samples;
    s_cal_count = 0U;
    s_cal_sum_x = 0.0f;
    s_cal_sum_y = 0.0f;
    s_cal_sum_z = 0.0f;
    s_cal_sum_sq = 0.0f;
    s_cal_acc_mag_sum = 0.0f;
    s_cal_min_dps[0] = 1.0e9f;
    s_cal_min_dps[1] = 1.0e9f;
    s_cal_min_dps[2] = 1.0e9f;
    s_cal_max_dps[0] = -1.0e9f;
    s_cal_max_dps[1] = -1.0e9f;
    s_cal_max_dps[2] = -1.0e9f;
    s_bias_valid = 0U;
    s_stationary = 0U;
    s_static_ms = 0.0f;
    s_gyro_z_prev_dps = 0.0f;
    s_calibrating = 1U;
  }

  if (dt_s <= 0.0f) {
    return;
  }

  if (!imu_read_regs(ICM20602_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return;
  }

  s_acc_raw[0] = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
  s_acc_raw[1] = (int16_t)(((uint16_t)raw[2] << 8) | (uint16_t)raw[3]);
  s_acc_raw[2] = (int16_t)(((uint16_t)raw[4] << 8) | (uint16_t)raw[5]);
  s_temp_raw  = (int16_t)(((uint16_t)raw[6] << 8) | (uint16_t)raw[7]);
  s_gyro_raw[0] = (int16_t)(((uint16_t)raw[8] << 8) | (uint16_t)raw[9]);
  s_gyro_raw[1] = (int16_t)(((uint16_t)raw[10] << 8) | (uint16_t)raw[11]);
  s_gyro_raw[2] = (int16_t)(((uint16_t)raw[12] << 8) | (uint16_t)raw[13]);

  if ((s_gyro_raw[0] > IMU_GYRO_SATURATION_RAW) ||
      (s_gyro_raw[0] < -IMU_GYRO_SATURATION_RAW) ||
      (s_gyro_raw[1] > IMU_GYRO_SATURATION_RAW) ||
      (s_gyro_raw[1] < -IMU_GYRO_SATURATION_RAW) ||
      (s_gyro_raw[2] > IMU_GYRO_SATURATION_RAW) ||
      (s_gyro_raw[2] < -IMU_GYRO_SATURATION_RAW)) {
    ++s_gyro_sat_count;
  }

  if (s_calibrating) {
    float gx_dps = (float)s_gyro_raw[0] / ICM20602_GYRO_LSB_PER_DPS;
    float gy_dps = (float)s_gyro_raw[1] / ICM20602_GYRO_LSB_PER_DPS;
    float gz_dps = (float)s_gyro_raw[2] / ICM20602_GYRO_LSB_PER_DPS;
    float ax_g = (float)s_acc_raw[0] / ICM20602_ACC_LSB_PER_G;
    float ay_g = (float)s_acc_raw[1] / ICM20602_ACC_LSB_PER_G;
    float az_g = (float)s_acc_raw[2] / ICM20602_ACC_LSB_PER_G;
    float acc_mag = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));

    s_cal_sum_x += (float)s_gyro_raw[0];
    s_cal_sum_y += (float)s_gyro_raw[1];
    s_cal_sum_z += (float)s_gyro_raw[2];
    s_cal_sum_sq += (float)s_gyro_raw[2] * (float)s_gyro_raw[2];
    s_cal_acc_mag_sum += acc_mag;
    if (gx_dps < s_cal_min_dps[0]) { s_cal_min_dps[0] = gx_dps; }
    if (gy_dps < s_cal_min_dps[1]) { s_cal_min_dps[1] = gy_dps; }
    if (gz_dps < s_cal_min_dps[2]) { s_cal_min_dps[2] = gz_dps; }
    if (gx_dps > s_cal_max_dps[0]) { s_cal_max_dps[0] = gx_dps; }
    if (gy_dps > s_cal_max_dps[1]) { s_cal_max_dps[1] = gy_dps; }
    if (gz_dps > s_cal_max_dps[2]) { s_cal_max_dps[2] = gz_dps; }
    ++s_cal_count;
    if (s_cal_count >= s_cal_target) {
      float variance;
      float spread_x = s_cal_max_dps[0] - s_cal_min_dps[0];
      float spread_y = s_cal_max_dps[1] - s_cal_min_dps[1];
      float spread_z = s_cal_max_dps[2] - s_cal_min_dps[2];

      s_bias_x_raw = s_cal_sum_x / (float)s_cal_count;
      s_bias_y_raw = s_cal_sum_y / (float)s_cal_count;
      s_bias_raw = s_cal_sum_z / (float)s_cal_count;
      s_acc_mag_reference = s_cal_acc_mag_sum / (float)s_cal_count;
      if ((s_acc_mag_reference < 0.3f) || (s_acc_mag_reference > 3.0f)) {
        s_acc_mag_reference = 1.0f;
      }
      s_calibration_spread_dps = spread_x;
      if (spread_y > s_calibration_spread_dps) {
        s_calibration_spread_dps = spread_y;
      }
      if (spread_z > s_calibration_spread_dps) {
        s_calibration_spread_dps = spread_z;
      }
      variance = (s_cal_sum_sq / (float)s_cal_count) -
                 (s_bias_raw * s_bias_raw);
      if (variance < 0.0f) {
        variance = 0.0f;
      }
      s_bias_std_raw = sqrtf(variance);
      s_calibrating = 0U;
      s_bias_valid =
          (uint8_t)(s_calibration_spread_dps <= IMU_CALIB_MAX_SPREAD_DPS);
      s_stationary = 0U;
      s_static_ms = 0.0f;
      s_gyro_z_prev_dps = 0.0f;
      s_gyro_dps[0] = 0.0f;
      s_gyro_dps[1] = 0.0f;
      s_gyro_dps[2] = 0.0f;
    }
    return;
  }

  if (!s_bias_valid) {
    s_gyro_dps[0] = (float)s_gyro_raw[0] / ICM20602_GYRO_LSB_PER_DPS;
    s_gyro_dps[1] = (float)s_gyro_raw[1] / ICM20602_GYRO_LSB_PER_DPS;
    s_gyro_dps[2] = (float)s_gyro_raw[2] / ICM20602_GYRO_LSB_PER_DPS;
    return;
  }

  corrected_dps[0] = ((float)s_gyro_raw[0] - s_bias_x_raw) /
                     ICM20602_GYRO_LSB_PER_DPS;
  corrected_dps[1] = ((float)s_gyro_raw[1] - s_bias_y_raw) /
                     ICM20602_GYRO_LSB_PER_DPS;
  corrected_dps[2] = ((float)s_gyro_raw[2] - s_bias_raw) /
                     ICM20602_GYRO_LSB_PER_DPS;

  acc_mag = sqrtf(Imu_GetAccXG() * Imu_GetAccXG() +
                  Imu_GetAccYG() * Imu_GetAccYG() +
                  Imu_GetAccZG() * Imu_GetAccZG());
  still_now = (uint8_t)(
      (fabsf(corrected_dps[0]) < IMU_STATIC_XY_DPS) &&
      (fabsf(corrected_dps[1]) < IMU_STATIC_XY_DPS) &&
      (fabsf(corrected_dps[2]) < IMU_STATIC_Z_DPS) &&
      (fabsf(acc_mag - s_acc_mag_reference) < IMU_STATIC_ACC_G));

  if (still_now) {
    s_static_ms += dt_s * 1000.0f;
  } else {
    s_static_ms = 0.0f;
  }
  s_stationary = (uint8_t)(s_static_ms >= (float)IMU_STATIC_CONFIRM_MS);

  if (s_stationary) {
    float gain = dt_s / IMU_BIAS_TRACK_TAU_S;

    if (gain > 1.0f) {
      gain = 1.0f;
    }
    s_bias_x_raw += ((float)s_gyro_raw[0] - s_bias_x_raw) * gain;
    s_bias_y_raw += ((float)s_gyro_raw[1] - s_bias_y_raw) * gain;
    s_bias_raw += ((float)s_gyro_raw[2] - s_bias_raw) * gain;
    corrected_dps[0] = ((float)s_gyro_raw[0] - s_bias_x_raw) /
                       ICM20602_GYRO_LSB_PER_DPS;
    corrected_dps[1] = ((float)s_gyro_raw[1] - s_bias_y_raw) /
                       ICM20602_GYRO_LSB_PER_DPS;
    corrected_dps[2] = ((float)s_gyro_raw[2] - s_bias_raw) /
                       ICM20602_GYRO_LSB_PER_DPS;
  }

  for (i = 0U; i < 3U; ++i) {
    s_gyro_dps[i] = corrected_dps[i];
  }

  /* Trapezoidal integration removes half a sample period of lag. */
  s_yaw_mrad += IMU_YAW_SIGN * IMU_YAW_SCALE * 0.5f *
                (s_gyro_z_prev_dps + corrected_dps[2]) *
                (3.14159265358979f / 180.0f) * dt_s * 1000.0f;
  s_gyro_z_prev_dps = corrected_dps[2];

  {
    float ax_g = Imu_GetAccXG();
    float ay_g = Imu_GetAccYG();
    float az_g = Imu_GetAccZG();
    float acc_roll = atan2f(ay_g, az_g) *
                     (180.0f / 3.14159265358979f);
    float acc_pitch = atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g))) *
                      (180.0f / 3.14159265358979f);

    s_roll_deg = 0.98f * (s_roll_deg + s_gyro_dps[0] * dt_s) +
                 0.02f * acc_roll;
    s_pitch_deg = 0.98f * (s_pitch_deg + s_gyro_dps[1] * dt_s) +
                  0.02f * acc_pitch;
  }
}

float Imu_GetYawRad(void)
{
  return s_yaw_mrad * 0.001f;
}

float Imu_GetYawMrad(void)
{
  return s_yaw_mrad;
}

void Imu_ResetYaw(float yaw_mrad)
{
  s_yaw_mrad = yaw_mrad;
  s_gyro_z_prev_dps = 0.0f;
}

float Imu_GetGyroZDps(void)
{
  return s_gyro_dps[2];
}

float Imu_GetGyroXDps(void)
{
  return s_gyro_dps[0];
}

float Imu_GetGyroYDps(void)
{
  return s_gyro_dps[1];
}

float Imu_GetRollDeg(void)
{
  return s_roll_deg;
}

float Imu_GetPitchDeg(void)
{
  return s_pitch_deg;
}

float Imu_GetAccXG(void)
{
  return (float)s_acc_raw[0] / ICM20602_ACC_LSB_PER_G;
}

float Imu_GetAccYG(void)
{
  return (float)s_acc_raw[1] / ICM20602_ACC_LSB_PER_G;
}

float Imu_GetAccZG(void)
{
  return (float)s_acc_raw[2] / ICM20602_ACC_LSB_PER_G;
}

float Imu_GetTemperatureC(void)
{
  return ICM20602_TEMP_OFFSET_C +
         (float)s_temp_raw / ICM20602_TEMP_LSB_PER_C;
}

int16_t Imu_GetAccXRaw(void)
{
  return s_acc_raw[0];
}

int16_t Imu_GetAccYRaw(void)
{
  return s_acc_raw[1];
}

int16_t Imu_GetAccZRaw(void)
{
  return s_acc_raw[2];
}

int16_t Imu_GetTemperatureRaw(void)
{
  return s_temp_raw;
}

int16_t Imu_GetGyroZRaw(void)
{
  return s_gyro_raw[2];
}

int16_t Imu_GetGyroXRaw(void)
{
  return s_gyro_raw[0];
}

int16_t Imu_GetGyroYRaw(void)
{
  return s_gyro_raw[1];
}

float Imu_GetGyroZBiasRaw(void)
{
  return s_bias_raw;
}

float Imu_GetGyroXBiasDps(void)
{
  return s_bias_x_raw / ICM20602_GYRO_LSB_PER_DPS;
}

float Imu_GetGyroYBiasDps(void)
{
  return s_bias_y_raw / ICM20602_GYRO_LSB_PER_DPS;
}

float Imu_GetGyroZBiasDps(void)
{
  return s_bias_raw / ICM20602_GYRO_LSB_PER_DPS;
}

uint8_t Imu_IsStationary(void)
{
  return s_stationary;
}

uint16_t Imu_GetCalibrationSamples(void)
{
  return s_cal_target;
}

float Imu_GetCalibrationNoiseDps(void)
{
  return s_bias_std_raw / ICM20602_GYRO_LSB_PER_DPS;
}

float Imu_GetStationaryDeadbandDps(void)
{
  return IMU_STATIC_Z_DPS;
}

float Imu_GetAccMagReference(void)
{
  return s_acc_mag_reference;
}

float Imu_GetCalibrationSpreadDps(void)
{
  return s_calibration_spread_dps;
}

uint32_t Imu_GetGyroSaturationCount(void)
{
  return s_gyro_sat_count;
}
