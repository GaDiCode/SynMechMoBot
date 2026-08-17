#include "ImuSensor.h"

// ================== CONFIG ==================
#define MPU6050_DLPF_44HZ      0x03
#define MPU6050_SAMPLE_125HZ   0x07

// ================== REGISTER ==================
#define SMPLRT_DIV_REG     0x19
#define CONFIG_REG         0x1A
#define GYRO_CONFIG_REG    0x1B
#define ACCEL_CONFIG_REG   0x1C
#define ACCEL_XOUT_H_REG   0x3B
#define PWR_MGMT_1_REG     0x6B

ImuSensor::ImuSensor() {
    i2c_bus = nullptr;
    last_time = 0;
    yaw_cont = 0.0f;
    gz_bias = 0.0f;
    gx_f = gy_f = gz_f = 0.0f;

    mpu_data.GyroOffsetX = 0.0f;
    mpu_data.GyroOffsetY = 0.0f;
    mpu_data.GyroOffsetZ = 0.0f;
}

bool ImuSensor::init(uint8_t i2c_address, uint32_t sda_pin, uint32_t scl_pin) {
    i2c_addr = i2c_address;
    if (i2c_bus == nullptr) {
        i2c_bus = new TwoWire(sda_pin, scl_pin);
    }
    i2c_bus->begin();

    // Wake up
    i2c_bus->beginTransmission(i2c_addr);
    i2c_bus->write(PWR_MGMT_1_REG);
    i2c_bus->write(0x00);
    if (i2c_bus->endTransmission(true) != 0) {
        return false;
    }

    // DLPF
    i2c_bus->beginTransmission(i2c_addr);
    i2c_bus->write(CONFIG_REG);
    i2c_bus->write(MPU6050_DLPF_44HZ);
    i2c_bus->endTransmission(true);

    // Sample rate
    i2c_bus->beginTransmission(i2c_addr);
    i2c_bus->write(SMPLRT_DIV_REG);
    i2c_bus->write(MPU6050_SAMPLE_125HZ);
    i2c_bus->endTransmission(true);

    // Accel config (+/- 2g is 0x00)
    i2c_bus->beginTransmission(i2c_addr);
    i2c_bus->write(ACCEL_CONFIG_REG);
    i2c_bus->write(0x00);
    i2c_bus->endTransmission(true);

    // Gyro config (+/- 250deg/s is 0x00)
    i2c_bus->beginTransmission(i2c_addr);
    i2c_bus->write(GYRO_CONFIG_REG);
    i2c_bus->write(0x00);
    i2c_bus->endTransmission(true);

    initMadgwick(0.05f);

    calibrateGyro(500);

    last_time = millis();
    return true;
}

void ImuSensor::readAll() {
    i2c_bus->beginTransmission(i2c_addr);
    i2c_bus->write(ACCEL_XOUT_H_REG);
    i2c_bus->endTransmission(false);
    i2c_bus->requestFrom((uint8_t)i2c_addr, (uint8_t)14, (uint8_t)1);

    if (i2c_bus->available() >= 14) {
        mpu_data.Accel_X_RAW = (i2c_bus->read() << 8) | i2c_bus->read();
        mpu_data.Accel_Y_RAW = (i2c_bus->read() << 8) | i2c_bus->read();
        mpu_data.Accel_Z_RAW = (i2c_bus->read() << 8) | i2c_bus->read();

        // Temperature (ignored)
        i2c_bus->read(); i2c_bus->read();

        mpu_data.Gyro_X_RAW  = (i2c_bus->read() << 8) | i2c_bus->read();
        mpu_data.Gyro_Y_RAW  = (i2c_bus->read() << 8) | i2c_bus->read();
        mpu_data.Gyro_Z_RAW  = (i2c_bus->read() << 8) | i2c_bus->read();
    }

    mpu_data.Ax = mpu_data.Accel_X_RAW / 16384.0f;
    mpu_data.Ay = mpu_data.Accel_Y_RAW / 16384.0f;
    mpu_data.Az = mpu_data.Accel_Z_RAW / 16384.0f;

    mpu_data.Gx = mpu_data.Gyro_X_RAW / 131.0f;
    mpu_data.Gy = mpu_data.Gyro_Y_RAW / 131.0f;
    mpu_data.Gz = mpu_data.Gyro_Z_RAW / 131.0f;
}

void ImuSensor::calibrateGyro(int samples) {
    float sx=0, sy=0, sz=0;
    
    // Warm up
    for(int i=0; i<100; i++) {
        readAll();
        delay(3);
    }

    for(int i=0; i<samples; i++) {
        readAll();
        sx += mpu_data.Gx;
        sy += mpu_data.Gy;
        sz += mpu_data.Gz;
        delay(3);
    }

    mpu_data.GyroOffsetX = sx/samples;
    mpu_data.GyroOffsetY = sy/samples;
    mpu_data.GyroOffsetZ = sz/samples;
}

void ImuSensor::update() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0f;
    if (dt == 0.0f) dt = 0.01f;
    last_time = current_time;

    readAll();

    float gx = mpu_data.Gx - mpu_data.GyroOffsetX;
    float gy = mpu_data.Gy - mpu_data.GyroOffsetY;
    float gz = mpu_data.Gz - mpu_data.GyroOffsetZ;

    // Low-pass filter
    gx_f = (1.0f - LPF_ALPHA) * gx_f + LPF_ALPHA * gx;
    gy_f = (1.0f - LPF_ALPHA) * gy_f + LPF_ALPHA * gy;
    gz_f = (1.0f - LPF_ALPHA) * gz_f + LPF_ALPHA * gz;

    gx = gx_f;
    gy = gy_f;
    gz = gz_f;

    // Deadzone
    if (fabsf(gx) < DEADZONE_GYRO) gx = 0;
    if (fabsf(gy) < DEADZONE_GYRO) gy = 0;
    if (fabsf(gz) < DEADZONE_GYRO) gz = 0;

    float acc_norm = sqrtf(mpu_data.Ax*mpu_data.Ax + mpu_data.Ay*mpu_data.Ay + mpu_data.Az*mpu_data.Az);

    // Auto bias
    if (fabsf(gx)<0.1f && fabsf(gy)<0.1f && fabsf(gz)<0.1f &&
        acc_norm > 0.99f && acc_norm < 1.01f)
    {
        gz_bias = 0.999f * gz_bias + 0.001f * gz;
    }

    gz -= gz_bias;

    // Adaptive beta
    madgwick.beta = (acc_norm > 0.98f && acc_norm < 1.02f) ? 0.05f : 0.01f;

    updateMadgwick(gx, gy, gz, mpu_data.Ax, mpu_data.Ay, mpu_data.Az, dt);

    float roll, pitch, yaw;
    getEuler(&roll, &pitch, &yaw);

    yaw_cont += gz * dt;

    static float r = 0, p = 0;
    r = 0.9f * r + 0.1f * roll;
    p = 0.9f * p + 0.1f * pitch;

    mpu_data.Roll  = r;
    mpu_data.Pitch = p;
    mpu_data.Yaw   = yaw_cont;
}

float ImuSensor::getYaw() {
    return yaw_cont;
}

void ImuSensor::resetYaw() {
    yaw_cont = 0.0f;
}

void ImuSensor::initMadgwick(float beta) {
    madgwick.q0 = 1.0f;
    madgwick.q1 = 0.0f;
    madgwick.q2 = 0.0f;
    madgwick.q3 = 0.0f;
    madgwick.beta = beta;
}

void ImuSensor::updateMadgwick(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float q0 = madgwick.q0, q1 = madgwick.q1, q2 = madgwick.q2, q3 = madgwick.q3;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0) return;

    ax /= norm; ay /= norm; az /= norm;

    gx *= M_PI/180.0f;
    gy *= M_PI/180.0f;
    gz *= M_PI/180.0f;

    float f1 = 2.0f*(q1*q3 - q0*q2) - ax;
    float f2 = 2.0f*(q0*q1 + q2*q3) - ay;
    float f3 = 2.0f*(0.5f - q1*q1 - q2*q2) - az;

    float s0 = -2.0f*q2*f1 + 2.0f*q1*f2;
    float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3;
    float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3;
    float s3 =  2.0f*q1*f1 + 2.0f*q2*f2;

    norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm == 0) return;

    s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - madgwick.beta*s0;
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - madgwick.beta*s1;
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - madgwick.beta*s2;
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - madgwick.beta*s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    madgwick.q0 = q0/norm;
    madgwick.q1 = q1/norm;
    madgwick.q2 = q2/norm;
    madgwick.q3 = q3/norm;
}

void ImuSensor::getEuler(float *roll, float *pitch, float *yaw) {
    float q0 = madgwick.q0, q1 = madgwick.q1, q2 = madgwick.q2, q3 = madgwick.q3;

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f/M_PI;
    *pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f/M_PI;
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f/M_PI;
}