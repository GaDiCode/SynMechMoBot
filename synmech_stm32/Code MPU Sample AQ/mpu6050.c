/**
 * @file    mpu6050.c
 * @brief   MPU6050 Driver + Sensor Fusion
 * @author  Tong Van Quynh
 */

#include "mpu6050.h"

// ================== CONFIG ==================
#define MPU6050_ADDR           0xD0
#define MPU6050_DLPF_44HZ      0x03
#define MPU6050_SAMPLE_125HZ   0x07

// ================== REGISTER ==================
#define SMPLRT_DIV_REG     0x19
#define CONFIG_REG         0x1A
#define GYRO_CONFIG_REG    0x1B
#define ACCEL_CONFIG_REG   0x1C
#define ACCEL_XOUT_H_REG   0x3B
#define PWR_MGMT_1_REG     0x6B

// ================== FILTER PARAM ==================
#define LPF_ALPHA          0.3f
#define DEADZONE_GYRO      0.15f

// ================== GLOBAL ==================
static Madgwick_t madgwick;


// ================== MADGWICK ==================
void Madgwick_Init(Madgwick_t *mad, float beta)
{
    mad->q0 = 1.0f;
    mad->q1 = 0.0f;
    mad->q2 = 0.0f;
    mad->q3 = 0.0f;
    mad->beta = beta;
}

void Madgwick_Update(Madgwick_t *mad,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
    float q0 = mad->q0, q1 = mad->q1, q2 = mad->q2, q3 = mad->q3;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0) return;

    ax /= norm; ay /= norm; az /= norm;

    gx *= M_PI/180.0f;
    gy *= M_PI/180.0f;
    gz *= M_PI/180.0f;

    float f1 = 2*(q1*q3 - q0*q2) - ax;
    float f2 = 2*(q0*q1 + q2*q3) - ay;
    float f3 = 2*(0.5f - q1*q1 - q2*q2) - az;

    float s0 = -2*q2*f1 + 2*q1*f2;
    float s1 =  2*q3*f1 + 2*q0*f2 - 4*q1*f3;
    float s2 = -2*q0*f1 + 2*q3*f2 - 4*q2*f3;
    float s3 =  2*q1*f1 + 2*q2*f2;

    norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm == 0) return;

    s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - mad->beta*s0;
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - mad->beta*s1;
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - mad->beta*s2;
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - mad->beta*s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    mad->q0 = q0/norm;
    mad->q1 = q1/norm;
    mad->q2 = q2/norm;
    mad->q3 = q3/norm;
}

void Madgwick_GetEuler(Madgwick_t *mad,
                       float *roll, float *pitch, float *yaw)
{
    float q0 = mad->q0, q1 = mad->q1, q2 = mad->q2, q3 = mad->q3;

    *roll  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180/M_PI;
    *pitch = asinf(2*(q0*q2 - q3*q1)) * 180/M_PI;
    *yaw   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180/M_PI;
}


// ================== INIT ==================
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data = 0;

    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);

    data = MPU6050_DLPF_44HZ;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 100);

    data = MPU6050_SAMPLE_125HZ;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);

    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);

    Madgwick_Init(&madgwick, 0.05f);

    return 0;
}


// ================== READ ==================
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    uint8_t data[14];

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 14, 100);

    mpu->Accel_X_RAW = (int16_t)(data[0]<<8 | data[1]);
    mpu->Accel_Y_RAW = (int16_t)(data[2]<<8 | data[3]);
    mpu->Accel_Z_RAW = (int16_t)(data[4]<<8 | data[5]);

    mpu->Gyro_X_RAW  = (int16_t)(data[8]<<8 | data[9]);
    mpu->Gyro_Y_RAW  = (int16_t)(data[10]<<8 | data[11]);
    mpu->Gyro_Z_RAW  = (int16_t)(data[12]<<8 | data[13]);

    mpu->Ax = mpu->Accel_X_RAW / 16384.0f;
    mpu->Ay = mpu->Accel_Y_RAW / 16384.0f;
    mpu->Az = mpu->Accel_Z_RAW / 16384.0f;

    mpu->Gx = mpu->Gyro_X_RAW / 131.0f;
    mpu->Gy = mpu->Gyro_Y_RAW / 131.0f;
    mpu->Gz = mpu->Gyro_Z_RAW / 131.0f;
}


// ================== UPDATE ==================
void MPU6050_Update(MPU6050_t *mpu, float dt)
{
    static float gx_f = 0, gy_f = 0, gz_f = 0;
    static float gz_bias = 0;
    static float yaw_cont = 0;

    float gx = mpu->Gx - mpu->GyroOffsetX;
    float gy = mpu->Gy - mpu->GyroOffsetY;
    float gz = mpu->Gz - mpu->GyroOffsetZ;

    // Low-pass filter
    gx_f = (1-LPF_ALPHA)*gx_f + LPF_ALPHA*gx;
    gy_f = (1-LPF_ALPHA)*gy_f + LPF_ALPHA*gy;
    gz_f = (1-LPF_ALPHA)*gz_f + LPF_ALPHA*gz;

    gx = gx_f;
    gy = gy_f;
    gz = gz_f;

    // Deadzone
    if (fabsf(gx) < DEADZONE_GYRO) gx = 0;
    if (fabsf(gy) < DEADZONE_GYRO) gy = 0;
    if (fabsf(gz) < DEADZONE_GYRO) gz = 0;

    float acc_norm = sqrtf(mpu->Ax*mpu->Ax + mpu->Ay*mpu->Ay + mpu->Az*mpu->Az);

    // Auto bias
    if (fabsf(gx)<0.1f && fabsf(gy)<0.1f && fabsf(gz)<0.1f &&
        acc_norm > 0.99f && acc_norm < 1.01f)
    {
        gz_bias = 0.999f * gz_bias + 0.001f * gz;
    }

    gz -= gz_bias;

    // Adaptive beta
    madgwick.beta = (acc_norm > 0.98f && acc_norm < 1.02f) ? 0.05f : 0.01f;

    Madgwick_Update(&madgwick, gx, gy, gz, mpu->Ax, mpu->Ay, mpu->Az, dt);

    float roll, pitch, yaw;
    Madgwick_GetEuler(&madgwick, &roll, &pitch, &yaw);

    // Infinite yaw
    yaw_cont += gz * dt;

    // Smooth
    static float r = 0, p = 0;
    r = 0.9f * r + 0.1f * roll;
    p = 0.9f * p + 0.1f * pitch;

    mpu->Roll  = r;
    mpu->Pitch = p;
    mpu->Yaw   = yaw_cont;
}


// ================== CALIB ==================
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu)
{
    float sx=0, sy=0, sz=0;
    int samples = 2000;

    for(int i=0; i<samples; i++)
    {
        MPU6050_Read_All(hi2c, mpu);
        sx += mpu->Gx;
        sy += mpu->Gy;
        sz += mpu->Gz;
    }

    mpu->GyroOffsetX = sx/samples;
    mpu->GyroOffsetY = sy/samples;
    mpu->GyroOffsetZ = sz/samples;
}