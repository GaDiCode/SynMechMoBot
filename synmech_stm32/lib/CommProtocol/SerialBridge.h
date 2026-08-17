#ifndef SERIAL_BRIDGE_H
#define SERIAL_BRIDGE_H
#include <Arduino.h>

class SerialBridge {
private:
    HardwareSerial* serial_port;
    String input_buffer;

public:
    enum CommandType { CMD_NONE, CMD_MOTOR, CMD_CALIBRATE, CMD_STOP, CMD_PS2, CMD_SERIAL };

    SerialBridge(HardwareSerial* port);
    void init(long baud_rate);
    
    // Đọc liên tục, trả về true nếu nhận xong 1 lệnh đầy đủ (có dấu \n)
    bool readCommand(int &left_pwm_out, int &right_pwm_out); 
    CommandType readCommandEx(int &left_pwm_out, int &right_pwm_out);
    
    // Gửi dữ liệu góc xoay (Yaw) từ IMU lên cho Orange Pi
    void sendFeedback(float yaw_angle); 
    void sendTelemetry(float yaw);
    void sendAck(const char* type);
    void sendError(int code);
};
#endif