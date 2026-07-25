#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "SerialBridge.h"
#include "ImuSensor.h"

#ifdef USE_ENCODER
    #include "Encoder.h"
#endif

MotorDriver motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
MotorDriver motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
HardwareSerial SerialUART1(PA10, PA9);
SerialBridge orangePiComms(&SerialUART1);

ImuSensor mpu1;
#ifdef USE_DUAL_IMU
    ImuSensor mpu2;
#endif

#ifdef USE_ENCODER
    Encoder encLeft(ENCODER_RESOLUTION, GEAR_RATIO, WHEEL_RADIUS_M);
    Encoder encRight(ENCODER_RESOLUTION, GEAR_RATIO, WHEEL_RADIUS_M);

    void ISR_LeftEncoder() {
        if (digitalRead(ENC_LEFT_B) == HIGH) {
            encLeft.updateTick(1);
        } else {
            encLeft.updateTick(-1);
        }
    }

    void ISR_RightEncoder() {
        if (digitalRead(ENC_RIGHT_B) == HIGH) {
            encRight.updateTick(1);
        } else {
            encRight.updateTick(-1);
        }
    }
#endif

int target_pwm_left = 0;
int target_pwm_right = 0;
unsigned long last_serial_send_time = 0;

void setup() {
    motorLeft.init();
    motorRight.init();
    orangePiComms.init(115200);

    mpu1.init(MPU1_ADDRESS, MPU1_SDA, MPU1_SCL);

    #ifdef USE_DUAL_IMU
        mpu2.init(MPU2_ADDRESS, MPU2_SDA, MPU2_SCL);
    #endif

    #ifdef USE_ENCODER
        pinMode(ENC_LEFT_A, INPUT_PULLUP);
        pinMode(ENC_LEFT_B, INPUT_PULLUP);
        pinMode(ENC_RIGHT_A, INPUT_PULLUP);
        pinMode(ENC_RIGHT_B, INPUT_PULLUP);
        
        attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), ISR_LeftEncoder, RISING);
        attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), ISR_RightEncoder, RISING);
    #endif
}

void loop() {
    mpu1.update();
    float final_yaw = mpu1.getYaw();

    #ifdef USE_DUAL_IMU
        mpu2.update();
        final_yaw = (mpu1.getYaw() + mpu2.getYaw()) / 2.0;
    #endif

    #ifdef USE_ENCODER
        // Bắt STM32 tự động tính toán vận tốc ra m/s
        encLeft.calculateVelocity();
        encRight.calculateVelocity();
    #endif

    if (orangePiComms.readCommand(target_pwm_left, target_pwm_right)) {
        motorLeft.setSpeed(target_pwm_left);
        motorRight.setSpeed(target_pwm_right);
    }

    if (millis() - last_serial_send_time >= 50) {
        last_serial_send_time = millis();

        #ifdef USE_ENCODER
            SerialUART1.print("YAW:");
            SerialUART1.print(final_yaw);
            SerialUART1.print(",V_LEFT:");
            SerialUART1.print(encLeft.getMeterSec());
            SerialUART1.print(",V_RIGHT:");
            SerialUART1.println(encRight.getMeterSec());
        #else
            // Nếu không có Encoder, dùng lại hàm gửi feedback cơ bản (Chỉ có góc YAW)
            orangePiComms.sendFeedback(final_yaw);
        #endif
    }
}