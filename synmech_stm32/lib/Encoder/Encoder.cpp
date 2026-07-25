#include "Encoder.h"

// STM32 tự động làm phép nhân khi khởi tạo đối tượng
Encoder::Encoder(float resolution, float ratio, float radius) {
    ticks = 0;
    last_ticks = 0;
    last_time = 0;
    
    enc_resolution = resolution;
    gear_ratio = ratio;
    
    // Mạch tự tính TỔNG SỐ XUNG trên 1 vòng bánh xe thực tế
    total_ppr = enc_resolution * gear_ratio; 
    
    wheel_radius = radius;
    
    velocity_rpm = 0.0;
    velocity_rad_s = 0.0;
    velocity_m_s = 0.0;
}

void Encoder::updateTick(int direction) {
    ticks += direction;
}

long Encoder::getTicks() {
    noInterrupts();
    long current_ticks = ticks;
    interrupts();
    return current_ticks;
}

void Encoder::resetTicks() {
    noInterrupts();
    ticks = 0;
    interrupts();
}

void Encoder::calculateVelocity() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    
    if (dt >= 0.05) { 
        long current_ticks = getTicks();
        long delta_ticks = current_ticks - last_ticks;
        
        // 1. Tính RPM (Chia cho total_ppr mà mạch vừa tự tính ở trên)
        float revs = (float)delta_ticks / total_ppr;
        velocity_rpm = (revs / dt) * 60.0;
        
        // 2. Tính Radian/s
        velocity_rad_s = velocity_rpm * (2.0 * PI / 60.0);
        
        // 3. Tính m/s
        velocity_m_s = velocity_rad_s * wheel_radius;

        last_ticks = current_ticks;
        last_time = current_time;
    }
}

float Encoder::getRPM() { return velocity_rpm; }
float Encoder::getRadSec() { return velocity_rad_s; }
float Encoder::getMeterSec() { return velocity_m_s; }