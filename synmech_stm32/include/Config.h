#ifndef CONFIG_H
#define CONFIG_H

// SERIAL COMMUNICATION
#define SERIAL_BAUD  115200
#define TELEMETRY_HZ 50

// MECHANICAL & KINEMATICS
#define TRACK_WIDTH       0.168f   // meters (distance between wheels)
#define WHEEL_RADIUS_M    0.0215f  // meters
#define MAX_LINEAR_VEL    0.22f    // m/s max velocity for PWM scaling
#define MAX_ANGULAR_VEL   1.5f     // rad/s max rotational velocity

// MOTOR'S PINS (TB6612FNG)
#define MOTOR_LEFT_PWM   PA15
#define MOTOR_LEFT_IN1   PA9
#define MOTOR_LEFT_IN2   PA10
#define MOTOR_RIGHT_PWM  PA13
#define MOTOR_RIGHT_IN1  PA14
#define MOTOR_RIGHT_IN2  PA15
#define STBY             PA8

// ENCODER MODE
// #define USE_ENCODER    // Comment out if NOT using Encoders
#ifdef USE_ENCODER
    #define ENCODER_RESOLUTION  11.0f
    #define GEAR_RATIO          50.0f
    
    // Choose pins with EXTI support
    #define ENC_LEFT_A   PB4
    #define ENC_LEFT_B   PB5
    #define ENC_RIGHT_A  PB8
    #define ENC_RIGHT_B  PB9
#endif

// IMU MPU6050
#define MPU1_ADDRESS 0x68
#define MPU1_SDA     PB11
#define MPU1_SCL     PB10

// #define USE_DUAL_IMU   // Comment out if NOT using Dual IMU
#ifdef USE_DUAL_IMU
    // Second MPU6050 (AD0 to 3.3V -> 0x69)
    #define MPU2_ADDRESS 0x69
    #define MPU2_SDA     PB11
    #define MPU2_SCL     PB10
#endif

// PS2 WIRELESS CONTROLLER
// #define USE_PS2    // Comment out if NOT using PS2
#ifdef USE_PS2
    #define PS2_DAT  PB15  // MISO
    #define PS2_CMD  PB13  // MOSI
    #define PS2_CLK  PB14  // SCK
    #define PS2_ATT  PB12  // CS
    
    // Joystick tuning
    #define PS2_DEAD_ZONE     30     // Raw joystick center ±30
    #define PS2_MAX_LINEAR    MAX_LINEAR_VEL
    #define PS2_MAX_ANGULAR   MAX_ANGULAR_VEL
    #define PS2_READ_HZ       50
#endif

#endif