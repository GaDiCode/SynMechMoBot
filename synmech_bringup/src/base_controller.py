#!/usr/bin/env python3
"""
ROS node for synmech_mobot base controller.
Bridges ROS and STM32 via Serial.
Computes odometry using IMU yaw (closed-loop) and commanded velocity (open-loop).
"""

import rospy
import serial
import threading
import math
import numpy as np
import yaml
import tf
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class BaseController:
    def __init__(self):
        rospy.init_node('base_controller')

        # ROS Parameters
        self.port = rospy.get_param('~port', '/dev/stm32')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.track_width = rospy.get_param('~track_width', 0.168)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0215)
        self.calibration_file = rospy.get_param('~calibration_file', 'calibration.yaml')
        self.input_mode = rospy.get_param('~input_mode', 'serial')

        # State Variables
        self.imu_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.last_vx = 0.0
        self.last_w = 0.0
        self.last_time = rospy.Time.now()

        # Calibration Data
        self.pwm_points = []
        self.vel_points = []
        self.load_calibration()

        # Serial Connection
        self.serial_lock = threading.Lock()
        self.serial_conn = None
        self.connect_serial()

        if self.serial_conn:
            self.set_input_mode(self.input_mode)

        # Publishers & Subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Start Serial Reader Thread
        self.running = True
        self.reader_thread = threading.Thread(target=self.serial_reader_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

        # Main Control Loop
        self.rate = rospy.Rate(50)  # 50 Hz
        self.main_loop()

    def load_calibration(self):
        try:
            with open(self.calibration_file, 'r') as f:
                data = yaml.safe_load(f)
                points = data.get('calibration_points', [])
                if points:
                    # Sort by velocity to ensure monotonic increasing for interp
                    points.sort(key=lambda p: p[1])
                    self.pwm_points = [float(p[0]) for p in points]
                    self.vel_points = [float(p[1]) for p in points]
                    rospy.loginfo(f"Loaded {len(points)} calibration points.")
                else:
                    rospy.logwarn("Calibration points missing in YAML.")
        except Exception as e:
            rospy.logerr(f"Failed to load calibration file: {e}")
            self.pwm_points = [0, 255]
            self.vel_points = [0.0, 0.5]

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1.0)
            rospy.loginfo(f"Connected to STM32 on {self.port}")
        except serial.SerialException as e:
            rospy.logerr(f"Serial connection failed: {e}")
            self.serial_conn = None

    def set_input_mode(self, mode):
        if mode == 'ps2':
            self.send_serial("PS2\n")
        else:
            self.send_serial("SER\n")

    def send_serial(self, msg):
        with self.serial_lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(msg.encode('utf-8'))
                except serial.SerialException:
                    rospy.logerr("Serial write failed. Connection lost.")
                    self.serial_conn.close()

    def vel_to_pwm(self, vel):
        """Convert velocity to PWM using calibration table."""
        if vel == 0:
            return 0
        direction = 1 if vel > 0 else -1
        # Interp requires x to be increasing (we map vel -> pwm)
        pwm = np.interp(abs(vel), self.vel_points, self.pwm_points)
        return int(direction * min(255, max(0, pwm)))

    def cmd_vel_callback(self, msg):
        self.last_vx = msg.linear.x
        self.last_w = msg.angular.z

        # Inverse Kinematics
        vl = self.last_vx - (self.last_w * self.track_width / 2.0)
        vr = self.last_vx + (self.last_w * self.track_width / 2.0)

        pwmL = self.vel_to_pwm(vl)
        pwmR = self.vel_to_pwm(vr)

        self.send_serial(f"M,{pwmL},{pwmR}\n")

    def serial_reader_loop(self):
        while self.running and not rospy.is_shutdown():
            if not self.serial_conn or not self.serial_conn.is_open:
                rospy.sleep(1.0)
                self.connect_serial()
                continue
            
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if not line:
                    continue
                
                parts = line.split(',')
                if parts[0] == 'D' and len(parts) >= 2:
                    yaw_deg = float(parts[1])
                    self.imu_yaw = math.radians(yaw_deg)
                elif parts[0] == 'A':
                    rospy.loginfo("STM32: Calibration ACK received.")
                elif parts[0] == 'E':
                    rospy.logerr(f"STM32 Error: {line}")
            except Exception as e:
                pass

    def main_loop(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # Update Odometry
            # Note: self.last_vx is open-loop commanded velocity
            # self.imu_yaw is closed-loop from STM32
            dx = self.last_vx * math.cos(self.imu_yaw) * dt
            dy = self.last_vx * math.sin(self.imu_yaw) * dt
            
            self.x += dx
            self.y += dy

            quat = tf.transformations.quaternion_from_euler(0, 0, self.imu_yaw)

            # Publish TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(t)

            # Publish Odometry
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = Quaternion(*quat)
            odom.twist.twist.linear.x = self.last_vx
            odom.twist.twist.angular.z = self.last_w
            self.odom_pub.publish(odom)

            # Publish Joint States
            js = JointState()
            js.header.stamp = current_time
            js.name = ['left_wheel_base', 'right_wheel_base']
            
            # Simple approximation of wheel positions (open loop)
            vl = self.last_vx - (self.last_w * self.track_width / 2.0)
            vr = self.last_vx + (self.last_w * self.track_width / 2.0)
            
            # For simplicity, we just publish velocity, and 0 position
            js.position = [0.0, 0.0] 
            js.velocity = [vl / self.wheel_radius, vr / self.wheel_radius]
            self.joint_pub.publish(js)

            self.rate.sleep()

    def stop(self):
        self.running = False
        self.send_serial("S\n")
        if self.serial_conn:
            self.serial_conn.close()

if __name__ == '__main__':
    try:
        controller = BaseController()
    except rospy.ROSInterruptException:
        pass