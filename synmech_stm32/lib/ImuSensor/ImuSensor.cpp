#include "ImuSensor.h"

ImuSensor::ImuSensor() {
    current_yaw = 0.0;
    gyro_z_offset = 0.0;
    last_time = 0;
    i2c_bus = nullptr; // Khởi tạo con trỏ an toàn
}

bool ImuSensor::init(uint8_t i2c_address, uint32_t sda_pin, uint32_t scl_pin) {
    // 1. Cấp phát linh động một bus I2C phần cứng mới dựa trên chân Config
    if (i2c_bus == nullptr) {
        i2c_bus = new TwoWire(sda_pin, scl_pin);
    }
    i2c_bus->begin();

    // 2. Truyền địa chỉ I2C và bus độc lập này vào thư viện Adafruit
    if (!mpu.begin(i2c_address, i2c_bus, 0)) {
        return false; 
    }

    // 3. Cấu hình thông số (Giữ nguyên)
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG); 
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

    // 4. Bù sai số
    calibrateGyro(500);

    last_time = millis();
    return true;
}

void ImuSensor::calibrateGyro(int samples) {
    float sum_gyro_z = 0;
    sensors_event_t a, g, temp;
    
    // Bỏ qua 100 mẫu đầu tiên vì cảm biến mới bật lên thường chưa ổn định
    for (int i = 0; i < 100; i++) {
        mpu.getEvent(&a, &g, &temp);
        delay(3);
    }

    // Tiến hành lấy trung bình số mẫu
    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &temp);
        sum_gyro_z += g.gyro.z;
        delay(3); // Chờ một chút để MPU cập nhật dữ liệu mới
    }
    
    // Lưu lại thông số lệch gốc (đơn vị: rad/s)
    gyro_z_offset = sum_gyro_z / samples;
}

void ImuSensor::update() {
    // 1. Tính toán thời gian trôi qua dt (giây)
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // 2. Lấy dữ liệu mới nhất
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 3. Lấy vận tốc góc trục Z (đã trừ đi sai số tĩnh lúc khởi động)
    float gyro_z_rad = g.gyro.z - gyro_z_offset;

    // 4. Bỏ qua nhiễu trắng nhỏ (Deadband) - Chống trôi khi xe đứng im
    if (abs(gyro_z_rad) < 0.01) {
        gyro_z_rad = 0;
    }

    // 5. Đổi từ rad/s sang độ/s
    float gyro_z_deg = gyro_z_rad * 57.2958; // 180 / PI

    // 6. Tích phân ra góc xoay (Yaw)
    current_yaw += gyro_z_deg * dt;
}

float ImuSensor::getYaw() {
    return current_yaw;
}

void ImuSensor::resetYaw() {
    current_yaw = 0.0;
}