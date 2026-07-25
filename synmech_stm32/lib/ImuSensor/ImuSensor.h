#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class ImuSensor {
private:
    Adafruit_MPU6050 mpu;
    TwoWire *i2c_bus;
    
    float current_yaw; 
    float gyro_z_offset; 
    unsigned long last_time; 

    void calibrateGyro(int samples = 500);

public:
    ImuSensor();

    // Nâng cấp: Truyền thẳng địa chỉ, chân SDA và chân SCL vào
    bool init(uint8_t i2c_address, uint32_t sda_pin, uint32_t scl_pin);

    void update();
    float getYaw();
    void resetYaw();
};

#endif