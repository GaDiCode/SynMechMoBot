#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include <Arduino.h>

class MotorDriver {
private:
    uint8_t pwm_pin;
    uint8_t in1_pin;
    uint8_t in2_pin;
    int current_speed; // Từ -255 đến 255

public:
    // Constructor (Hàm khởi tạo) nhận chân tín hiệu
    MotorDriver(uint8_t pwm, uint8_t in1, uint8_t in2);
    
    void init();
    void setSpeed(int speed); // Xử lý logic tiến, lùi, phanh tại đây
    int getSpeed(); 
};
#endif