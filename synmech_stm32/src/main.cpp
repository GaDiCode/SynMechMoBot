#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "SerialBridge.h"
#include "ImuSensor.h"
#include "Kinematics.h"

#ifdef USE_ENCODER
    #include "Encoder.h"
#endif

#ifdef USE_PS2
    #include "PS2X_lib.h"
#endif

MotorDriver motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2);
MotorDriver motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
HardwareSerial SerialUART1(PA10, PA9);
SerialBridge orangePiComms(&SerialUART1);

ImuSensor mpu1;
#ifdef USE_DUAL_IMU
    ImuSensor mpu2;
#endif

// Using MAX_LINEAR_VEL from Config (ensure we define it universally in Config if missing)
#ifndef MAX_LINEAR_VEL
#define MAX_LINEAR_VEL 0.22f
#endif

Kinematics kinematics(TRACK_WIDTH, MAX_LINEAR_VEL);

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

#ifdef USE_PS2
    PS2X ps2;
#endif

enum InputMode { MODE_SERIAL, MODE_PS2 };
InputMode currentMode = MODE_SERIAL;

int target_pwm_left = 0;
int target_pwm_right = 0;
unsigned long last_serial_send_time = 0;
unsigned long last_imu_time = 0;
unsigned long last_ps2_time = 0;

void setup() {
    digitalWrite(STBY, 1);
    motorLeft.init();
    motorRight.init();
    orangePiComms.init(SERIAL_BAUD);

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

    #ifdef USE_PS2
        // Setup PS2 controller, disable pressures and rumble
        int error = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
        if (error == 0) {
            currentMode = MODE_PS2;
        }
    #endif
}

void loop() {
    unsigned long current_time = millis();

    // 1. IMU Update (100Hz -> every 10ms)
    if (current_time - last_imu_time >= 10) {
        last_imu_time = current_time;
        mpu1.update();
        #ifdef USE_DUAL_IMU
            mpu2.update();
        #endif
    }
    
    float final_yaw = mpu1.getYaw();
    #ifdef USE_DUAL_IMU
        final_yaw = (mpu1.getYaw() + mpu2.getYaw()) / 2.0f;
    #endif

    #ifdef USE_ENCODER
        encLeft.calculateVelocity();
        encRight.calculateVelocity();
    #endif

    // 2. Serial Commands (always active)
    SerialBridge::CommandType cmd = orangePiComms.readCommandEx(target_pwm_left, target_pwm_right);
    if (cmd == SerialBridge::CMD_MOTOR) {
        currentMode = MODE_SERIAL;
        motorLeft.setSpeed(target_pwm_left);
        motorRight.setSpeed(target_pwm_right);
    } else if (cmd == SerialBridge::CMD_CALIBRATE) {
        mpu1.calibrateGyro(500);
        #ifdef USE_DUAL_IMU
            mpu2.calibrateGyro(500);
        #endif
        orangePiComms.sendAck("CALIBRATED");
    } else if (cmd == SerialBridge::CMD_STOP) {
        motorLeft.setSpeed(0);
        motorRight.setSpeed(0);
    } else if (cmd == SerialBridge::CMD_PS2) {
        currentMode = MODE_PS2;
    } else if (cmd == SerialBridge::CMD_SERIAL) {
        currentMode = MODE_SERIAL;
    }

    // 3. PS2 Controller Mode
    #ifdef USE_PS2
    if (currentMode == MODE_PS2) {
        if (current_time - last_ps2_time >= (1000 / PS2_READ_HZ)) {
            last_ps2_time = current_time;
            ps2.read_gamepad(false, 0);

            if (ps2.isEmergencyStop()) {
                motorLeft.setSpeed(0);
                motorRight.setSpeed(0);
            } else {
                float multiplier = ps2.getSpeedMultiplier();
                float vx = ps2.getRobotLinearVel(PS2_MAX_LINEAR, PS2_DEAD_ZONE) * multiplier;
                float wz = ps2.getRobotAngularVel(PS2_MAX_ANGULAR, PS2_DEAD_ZONE) * multiplier;

                float vL = 0, vR = 0;
                kinematics.inverseKinematics(vx, wz, vL, vR);

                int pwmL = kinematics.velocityToPWM(vL);
                int pwmR = kinematics.velocityToPWM(vR);

                motorLeft.setSpeed(pwmL);
                motorRight.setSpeed(pwmR);
            }
        }
    }
    #endif

    // 4. Telemetry (50Hz)
    if (current_time - last_serial_send_time >= (1000 / TELEMETRY_HZ)) {
        last_serial_send_time = current_time;

        #ifdef USE_ENCODER
            SerialUART1.print("YAW:");
            SerialUART1.print(final_yaw);
            SerialUART1.print(",V_LEFT:");
            SerialUART1.print(encLeft.getMeterSec());
            SerialUART1.print(",V_RIGHT:");
            SerialUART1.println(encRight.getMeterSec());
        #else
            orangePiComms.sendTelemetry(final_yaw);
        #endif
    }
}