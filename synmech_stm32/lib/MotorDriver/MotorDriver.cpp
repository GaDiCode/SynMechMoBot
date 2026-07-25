#include "MotorDriver.h"

// Constructor
MotorDriver::MotorDriver(uint8_t pwm, uint8_t in1, uint8_t in2) {
    this->pwm_pin = pwm;
    this->in1_pin = in1;
    this->in2_pin = in2;
    this->current_speed = 0;
}

void MotorDriver::init() {
    pinMode(pwm_pin, OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);
    
    setSpeed(0); 
}

void MotorDriver::setSpeed(int speed) {
    current_speed = speed;
    
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, speed);
    } 
    else if (speed < 0) {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, HIGH);
        analogWrite(pwm_pin, -speed);
    } 
    else {
        digitalWrite(in1_pin, LOW);
        digitalWrite(in2_pin, LOW);
        analogWrite(pwm_pin, 0);
    }
}

int MotorDriver::getSpeed() {
    return current_speed;
}