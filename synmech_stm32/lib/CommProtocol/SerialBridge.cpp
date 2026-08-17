#include "SerialBridge.h"

SerialBridge::SerialBridge(HardwareSerial* port) {
    this->serial_port = port;
    this->input_buffer = "";
}

void SerialBridge::init(long baud_rate) {
    serial_port->begin(baud_rate);
}

// Hàm đọc lệnh: Trả về true nếu đã nhận đủ 1 dòng lệnh hoàn chỉnh
bool SerialBridge::readCommand(int &left_pwm_out, int &right_pwm_out) {
    return (readCommandEx(left_pwm_out, right_pwm_out) == CMD_MOTOR);
}

SerialBridge::CommandType SerialBridge::readCommandEx(int &left_pwm_out, int &right_pwm_out) {
    while (serial_port->available()) {
        char c = (char)serial_port->read();
        
        if (c == '\n') {
            String line = input_buffer;
            line.trim();
            input_buffer = ""; 
            
            if (line.length() == 0) return CMD_NONE;

            if (line == "C") return CMD_CALIBRATE;
            if (line == "S") return CMD_STOP;
            if (line == "PS2") return CMD_PS2;
            if (line == "SER") return CMD_SERIAL;

            if (line.startsWith("M,")) {
                int firstComma = line.indexOf(',');
                int secondComma = line.indexOf(',', firstComma + 1);
                if (firstComma != -1 && secondComma != -1) {
                    left_pwm_out = line.substring(firstComma + 1, secondComma).toInt();
                    right_pwm_out = line.substring(secondComma + 1).toInt();
                    return CMD_MOTOR;
                }
            } else {
                // Backward compatibility: <left>,<right>
                int comma_idx = line.indexOf(',');
                if (comma_idx != -1) {
                    left_pwm_out = line.substring(0, comma_idx).toInt();
                    right_pwm_out = line.substring(comma_idx + 1).toInt();
                    return CMD_MOTOR;
                }
            }
            return CMD_NONE;
        } else {
            input_buffer += c;
        }
    }
    return CMD_NONE;
}

// Hàm gửi phản hồi từ MPU6050 lên ROS
void SerialBridge::sendFeedback(float yaw_angle) {
    serial_port->print("YAW:");
    serial_port->println(yaw_angle);
}

void SerialBridge::sendTelemetry(float yaw) {
    serial_port->print("D,");
    serial_port->println(yaw, 2);
}

void SerialBridge::sendAck(const char* type) {
    serial_port->print("A,");
    serial_port->println(type);
}

void SerialBridge::sendError(int code) {
    serial_port->print("E,");
    serial_port->println(code);
}