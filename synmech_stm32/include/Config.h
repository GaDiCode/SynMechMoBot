#ifndef CONFIG_H
#define CONFIG_H

// MPU1
#define MPU1_ADDRESS 0x68
#define MPU1_SDA     PB11
#define MPU1_SCL     PB10

#ifdef USE_DUAL_IMU
    // MPU thứ 2 (Nếu cắm chung dây với MPU1 thì khai báo trùng chân, đổi địa chỉ thành 0x69)
    // (Nếu cắm riêng rẽ sang bus khác thì khai báo chân khác, ví dụ PB7, PB6)
    #define MPU2_ADDRESS 0x69
    #define MPU2_SDA     PB11  // Đổi thành PB7 nếu cắm bus I2C1
    #define MPU2_SCL     PB10  // Đổi thành PB6 nếu cắm bus I2C1
#endif


#define ENCODER_RESOLUTION  11.0
#define GEAR_RATIO          50.0
#define WHEEL_RADIUS_M      0.0215
// MOTOR'S PINS
#define MOTOR_LEFT_PWM   PA1
#define MOTOR_LEFT_IN1   PA3
#define MOTOR_LEFT_IN2   PA4
#define MOTOR_RIGHT_PWM  PA2
#define MOTOR_RIGHT_IN1  PA5
#define MOTOR_RIGHT_IN2  PA6

// ENCODER MODE
// #define USE_ENCODER    // Comment dòng này lại nếu KHÔNG dùng Encoder
#ifdef USE_ENCODER
    // Chọn các chân có hỗ trợ Ngắt ngoài (EXTI)
    #define ENC_LEFT_A   PB4
    #define ENC_LEFT_B   PB5
    #define ENC_RIGHT_A  PB8
    #define ENC_RIGHT_B  PB9
#endif

// DUAL MPU6050
// MPU1 - Default address: 0x68
#define MPU1_ADDRESS 0x68

#ifdef USE_DUAL_IMU
    // MPU thứ 2: Cắm chung 2 dây I2C (PB10, PB11) cùng con MPU1.
    // NHƯNG CHÚ Ý: Chân AD0 của MPU thứ 2 phải hàn vào 3.3V => Địa chỉ 0x69
    #define MPU2_ADDRESS 0x69
#endif

#endif