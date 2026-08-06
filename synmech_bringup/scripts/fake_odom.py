#!/usr/bin/env python3
"""
fake_odom.py — Fake Odometry Node cho Phase 1 (Không cần STM32/Encoder)

CHỨC NĂNG:
  1. Phát TF: odom → base_footprint (để Gmapping biết xe đang ở đâu)
  2. Phát topic /odom (nav_msgs/Odometry)
  3. Phát topic /joint_states cho 2 bánh xe (fix lỗi TF tree)
  4. Lắng nghe /cmd_vel → tích phân tính vị trí giả (dead reckoning)

CÁCH DÙNG:
  - Phase 1: Bê xe bằng tay, Gmapping dùng scan matching để vẽ bản đồ.
             Odometry giả sẽ báo "xe đứng yên" (vì không ai gửi /cmd_vel),
             nhưng Gmapping vẫn vẽ được nhờ so sánh 2 bản quét lidar liên tiếp.
  - Nếu bạn chạy teleop trên Laptop, fake_odom sẽ tích phân /cmd_vel
    để ước lượng vị trí (open-loop, có sai số nhưng đủ để test).

THAY THẾ BỞI:
  - base_controller.py khi có STM32 + MPU6050 (Phase 2+)
"""

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class FakeOdom:
    def __init__(self):
        rospy.init_node('fake_odom', anonymous=False)

        # ====== THÔNG SỐ CƠ KHÍ (Lấy từ URDF robot_core.xacro) ======
        self.track_width = rospy.get_param('~track_width', 0.168)      # Khoảng cách 2 bánh (m)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0215)   # Bán kính bánh xe (m)

        # ====== BIẾN TRẠNG THÁI ODOMETRY ======
        self.x = 0.0          # Vị trí X (m) trong hệ tọa độ odom
        self.y = 0.0          # Vị trí Y (m)
        self.theta = 0.0      # Góc quay (rad)
        self.vx = 0.0         # Vận tốc tuyến tính hiện tại (m/s)
        self.vth = 0.0        # Vận tốc góc hiện tại (rad/s)
        self.last_time = rospy.Time.now()

        # Biến tính góc quay bánh xe (cho joint_states)
        self.left_wheel_pos = 0.0    # Góc quay tích lũy bánh trái (rad)
        self.right_wheel_pos = 0.0   # Góc quay tích lũy bánh phải (rad)

        # ====== ROS PUBLISHERS ======
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ====== ROS SUBSCRIBER ======
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        rospy.loginfo("[fake_odom] Node khởi động thành công!")
        rospy.loginfo("[fake_odom] Track width: %.3f m | Wheel radius: %.4f m",
                      self.track_width, self.wheel_radius)
        rospy.loginfo("[fake_odom] Đang phát TF odom→base_footprint + /joint_states")

    def cmd_vel_callback(self, msg):
        """Nhận lệnh vận tốc từ teleop keyboard"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        """Tính toán odometry và phát TF + topics mỗi chu kỳ"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if dt <= 0:
            return

        # ====== TÍCH PHÂN VỊ TRÍ (Dead Reckoning) ======
        # Đây là phép tính "open-loop": giả sử xe chạy đúng theo lệnh cmd_vel
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # ====== TÍNH GÓC QUAY BÁNH XE (cho joint_states) ======
        # Inverse kinematics: từ vận tốc xe → vận tốc từng bánh
        v_left = self.vx - (self.vth * self.track_width / 2.0)
        v_right = self.vx + (self.vth * self.track_width / 2.0)

        # Vận tốc góc bánh = vận tốc tuyến tính / bán kính
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Tích lũy góc quay
        self.left_wheel_pos += omega_left * dt
        self.right_wheel_pos += omega_right * dt

        # ====== PHÁT TF: odom → base_footprint ======
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            "base_footprint",    # child frame
            "odom"               # parent frame
        )

        # ====== PHÁT TOPIC /odom ======
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        # ====== PHÁT TOPIC /joint_states (Fix lỗi TF bánh xe) ======
        js = JointState()
        js.header.stamp = current_time
        js.name = ['left_wheel_base', 'right_wheel_base']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        js.velocity = [omega_left, omega_right]
        js.effort = []

        self.joint_pub.publish(js)

        self.last_time = current_time

    def run(self):
        """Vòng lặp chính — chạy ở 50Hz"""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = FakeOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass
