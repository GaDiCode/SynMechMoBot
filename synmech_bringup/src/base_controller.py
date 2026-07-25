#!/usr/bin/env python3
import rospy
import serial
import math
import threading
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

class SynmechBaseController:
    def __init__(self):
        rospy.init_node('synmech_base_controller', anonymous=False)

        # 1. CẤU HÌNH THÔNG SỐ CƠ KHÍ & SERIAL
        # Thay đổi cổng /dev/ttyUSB0 tùy thuộc vào cáp cắm thực tế
        self.serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baudrate', 115200)
        
        # Khoảng cách giữa 2 bánh xe (Track width) - Đơn vị: mét
        self.L = rospy.get_param('~track_width', 0.16) 
        
        # Hệ số quy đổi (Mapping) từ m/s sang PWM (0-255). 
        # Cần tinh chỉnh (Tune) con số này khi chạy thực tế ngoài đời.
        self.VEL_TO_PWM_RATIO = rospy.get_param('~vel_to_pwm', 200.0) 

        # 2. KHỞI TẠO BIẾN ODOMETRY
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # 3. KẾT NỐI UART (Bắt lỗi chuẩn Python 3)
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            rospy.loginfo(f"Đã kết nối UART thành công tới STM32 tại {self.serial_port}")
        except serial.SerialException as e:
            rospy.logerr(f"Lỗi cổng Serial: {e}")
            rospy.signal_shutdown("Không tìm thấy mạch STM32")

        # 4. ROS PUBLISHERS & SUBSCRIBERS
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # 5. MỞ LUỒNG (THREAD) RIÊNG ĐỂ ĐỌC SENSOR TỪ STM32
        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    # ==========================================
    # KHỐI 1: NHẬN LỆNH ROS -> TÍNH TOÁN -> GỬI XUỐNG STM32
    # ==========================================
    def cmd_vel_callback(self, msg):
        v = msg.linear.x    # Vận tốc tịnh tiến (m/s)
        w = msg.angular.z   # Vận tốc góc xoay (rad/s)

        # Động học ngược (Inverse Kinematics) cho xe vi sai
        v_left = v - (w * self.L / 2.0)
        v_right = v + (w * self.L / 2.0)

        # Mapping Open-loop: Chuyển m/s thành PWM (Do STM32 chưa chạy PID)
        pwm_left = int(v_left * self.VEL_TO_PWM_RATIO)
        pwm_right = int(v_right * self.VEL_TO_PWM_RATIO)

        # Ép khung giới hạn [-255, 255] an toàn
        pwm_left = max(min(pwm_left, 255), -255)
        pwm_right = max(min(pwm_right, 255), -255)

        # Đóng gói thành chuỗi Bytes chuẩn Python 3 và gửi đi
        # Định dạng gửi: "150,-100\n"
        command_str = f"{pwm_left},{pwm_right}\n"
        self.ser.write(command_str.encode('utf-8'))

    # ==========================================
    # KHỐI 2: ĐỌC STM32 -> TÍNH ODOMETRY -> BẮN LÊN ROS
    # ==========================================
    def serial_read_loop(self):
        rate = rospy.Rate(50) # Chạy luồng đọc ở tần số cao
        while not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    # Đọc và decode từ Bytes -> String
                    raw_data = self.ser.readline().decode('utf-8').strip()
                    self.parse_and_publish_odom(raw_data)
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"Lỗi đọc UART: {e}")
            rate.sleep()

    def parse_and_publish_odom(self, data_str):
        # Kỳ vọng nhận được: "YAW:15.5,V_LEFT:0.2,V_RIGHT:0.2"
        # Hoặc nếu không có encoder: "YAW:15.5"
        
        yaw_deg = 0.0
        v_l = 0.0
        v_r = 0.0

        parts = data_str.split(',')
        for part in parts:
            if part.startswith("YAW:"):
                yaw_deg = float(part.split(':')[1])
            elif part.startswith("V_LEFT:"):
                v_l = float(part.split(':')[1])
            elif part.startswith("V_RIGHT:"):
                v_r = float(part.split(':')[1])

        # Tính toán Động học thuận (Forward Kinematics)
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Vận tốc tuyến tính trung bình của tâm xe
        v_robot = (v_l + v_r) / 2.0
        
        # Đổi độ sang Radian chuẩn của ROS
        self.theta = math.radians(yaw_deg) 

        # Tích phân vị trí (x, y) trên bản đồ
        delta_x = v_robot * math.cos(self.theta) * dt
        delta_y = v_robot * math.sin(self.theta) * dt

        self.x += delta_x
        self.y += delta_y
        self.last_time = current_time

        # Ép kiểu Quaternion (Toán học hình học không gian)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # 1. BẮN TF BẢN ĐỒ (odom -> base_link)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # 2. BẮN TOPIC /odom CHO NAVIGATION DÙNG
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.twist.twist.linear.x = v_robot
        odom.twist.twist.angular.z = v_robot # Nếu cần w thực tế có thể tính (v_r - v_l) / L

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        controller = SynmechBaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass