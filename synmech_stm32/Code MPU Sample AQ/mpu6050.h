/**
 * @file    mpu6050.h
 * @brief   MPU6050 Driver + Sensor Fusion (Madgwick)
 * @author  Tong Van Quynh
 */

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f1xx_hal.h"
#include <math.h>

// ================== DATA STRUCT ==================
typedef struct {
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    int16_t Gyro_X_RAW,  Gyro_Y_RAW,  Gyro_Z_RAW;

    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    float Roll, Pitch, Yaw;

    float GyroOffsetX;
    float GyroOffsetY;
    float GyroOffsetZ;

    float gx_t, gy_t, gz_t;

} MPU6050_t;


// ================== MADGWICK ==================
typedef struct {
    float q0, q1, q2, q3;
    float beta;
} Madgwick_t;


// ================== FUNCTION ==================
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);
void MPU6050_Update(MPU6050_t *mpu, float dt);
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu);

// Madgwick
void Madgwick_Init(Madgwick_t *mad, float beta);
void Madgwick_Update(Madgwick_t *mad,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt);

void Madgwick_GetEuler(Madgwick_t *mad,
                       float *roll, float *pitch, float *yaw);

#endif