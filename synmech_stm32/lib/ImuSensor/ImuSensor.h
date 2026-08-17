#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
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

} MPU6050_Data;

// ================== MADGWICK ==================
typedef struct {
    float q0, q1, q2, q3;
    float beta;
} Madgwick_Data;

class ImuSensor {
private:
    TwoWire *i2c_bus;
    uint8_t  i2c_addr;

    MPU6050_Data mpu_data;
    Madgwick_Data madgwick;

    unsigned long last_time; 
    
    float yaw_cont;
    float gz_bias;
    float gx_f, gy_f, gz_f;

    static constexpr float DEADZONE_GYRO = 0.15f;
    static constexpr float LPF_ALPHA = 0.3f;


    void readAll();
    void initMadgwick(float beta);
    void updateMadgwick(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void getEuler(float *roll, float *pitch, float *yaw);

public:
    ImuSensor();
    void calibrateGyro(int samples = 500);

    // Nâng cấp: Truyền thẳng địa chỉ, chân SDA và chân SCL vào
    bool init(uint8_t i2c_address, uint32_t sda_pin, uint32_t scl_pin);

    void update();
    float getYaw();
    void resetYaw();
};

#endif