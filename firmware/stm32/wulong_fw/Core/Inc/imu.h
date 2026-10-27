/**
  ******************************************************************************
  * @file    imu.h
  * @brief   ICM-20602 IMU driver and yaw integration.
  ******************************************************************************
  */

#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Initialise SPI2 + CS and configure the ICM-20602 for:
 *   gyro +/-500 dps, accel +/-4 g, 1 kHz ODR, ~92 Hz DLPF.
 * Returns 1 on success, 0 if the WHO_AM_I check fails.
 */
uint8_t Imu_Init(void);

/* True once Imu_Init() has configured a sensor that answers 0x12. */
uint8_t Imu_IsReady(void);

/*
 * Request a stationary gyro-Z bias calibration.  Safe to call from any task:
 * the next Imu_Update() picks the request up, then the following `samples`
 * updates only accumulate data while yaw is held.  Keep the chassis still.
 */
void Imu_StartBiasCalibration(uint16_t samples);
uint8_t Imu_CalibrationDone(void);
uint8_t Imu_IsCalibrating(void);

/*
 * Read one sample and integrate gyro-Z into yaw.  dt_s is the time since the
 * previous call.  Call every control tick (1 kHz).
 */
void Imu_Update(float dt_s);

/* Current integrated yaw, radians / milliradians (CCW positive). */
float Imu_GetYawRad(void);
float Imu_GetYawMrad(void);

/* Zero the integrated yaw without touching the gyro bias. */
void Imu_ResetYaw(float yaw_mrad);

/* Diagnostics. */
float Imu_GetAccXG(void);
float Imu_GetAccYG(void);
float Imu_GetAccZG(void);
float Imu_GetTemperatureC(void);
int16_t Imu_GetAccXRaw(void);
int16_t Imu_GetAccYRaw(void);
int16_t Imu_GetAccZRaw(void);
int16_t Imu_GetTemperatureRaw(void);
float Imu_GetGyroXDps(void);
float Imu_GetGyroYDps(void);
float Imu_GetGyroZDps(void);
float Imu_GetRollDeg(void);
float Imu_GetPitchDeg(void);
int16_t Imu_GetGyroXRaw(void);
int16_t Imu_GetGyroYRaw(void);
int16_t Imu_GetGyroZRaw(void);
float Imu_GetGyroXBiasDps(void);
float Imu_GetGyroYBiasDps(void);
float Imu_GetGyroZBiasDps(void);
float Imu_GetGyroZBiasRaw(void);
float Imu_GetAccMagReference(void);
float Imu_GetCalibrationSpreadDps(void);
uint32_t Imu_GetGyroSaturationCount(void);
uint8_t Imu_IsStationary(void);
uint16_t Imu_GetCalibrationSamples(void);
float Imu_GetCalibrationNoiseDps(void);
float Imu_GetStationaryDeadbandDps(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
