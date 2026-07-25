#ifndef ENCODER_H
#define ENCODER_H
#include <Arduino.h>

class Encoder {
private:
    volatile long ticks; 
    long last_ticks;
    unsigned long last_time;
    
    // Các biến cấu hình cơ khí
    float enc_resolution;
    float gear_ratio;
    float total_ppr; // STM32 sẽ tự tính và lưu vào đây (Total Pulses Per Revolution)
    float wheel_radius;

    // Các biến lưu trữ kết quả
    float velocity_rpm;
    float velocity_rad_s;
    float velocity_m_s;

public:
    // Cập nhật Constructor để nhận 3 tham số rành mạch
    Encoder(float resolution, float ratio, float radius);
    
    void updateTick(int direction);
    long getTicks();
    void resetTicks();
    
    void calculateVelocity(); 
    float getRPM();
    float getRadSec();
    float getMeterSec();
};
#endif