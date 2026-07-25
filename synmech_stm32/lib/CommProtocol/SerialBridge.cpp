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
    while (serial_port->available()) {
        char c = (char)serial_port->read();
        
        // Ký tự \n (Enter) là dấu hiệu kết thúc 1 gói tin
        if (c == '\n') {
            int comma_idx = input_buffer.indexOf(',');
            
            // Nếu tìm thấy dấu phẩy, tiến hành tách số
            if (comma_idx != -1) {
                String left_str = input_buffer.substring(0, comma_idx);
                String right_str = input_buffer.substring(comma_idx + 1);
                
                left_pwm_out = left_str.toInt();
                right_pwm_out = right_str.toInt();
            }
            
            // Xóa buffer để chuẩn bị đón gói tin tiếp theo
            input_buffer = ""; 
            return true; 
        } 
        else {
            // Nếu chưa hết dòng thì cứ cộng dồn ký tự vào chuỗi
            input_buffer += c;
        }
    }
    return false;
}

// Hàm gửi phản hồi từ MPU6050 lên ROS
void SerialBridge::sendFeedback(float yaw_angle) {
    serial_port->print("YAW:");
    serial_port->println(yaw_angle);
}