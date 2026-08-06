#!/usr/bin/env python3
"""
fake_odom.py - Fake Odometry Node for Phase 1 (No STM32/Encoder required)

FEATURES:
  1. Publishes TF: odom -> base_footprint (provides localization for Gmapping)
  2. Publishes topic /odom (nav_msgs/Odometry)
  3. Publishes topic /joint_states for 2 wheels (resolves TF tree error)
  4. Subscribes to /cmd_vel -> integrates to estimate fake position (dead reckoning)

USAGE:
  - Phase 1: Manually move the robot. Gmapping relies on scan matching to build the map.
             Fake odometry will report "robot stationary" (no /cmd_vel input),
             but Gmapping can still map by matching consecutive Lidar scans.
  - If using a teleop node on the Laptop, fake_odom integrates /cmd_vel
    to estimate position (open-loop, contains drift but sufficient for initial testing).

DEPRECATION:
  - Replace with base_controller.py once STM32 + MPU6050 are integrated (Phase 2+)
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

        # ====== MECHANICAL PARAMETERS (Loaded from URDF robot_core.xacro) ======
        self.track_width = rospy.get_param('~track_width', 0.168)      # Distance between wheels (m)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0215)   # Wheel radius (m)

        # ====== ODOMETRY STATE VARIABLES ======
        self.x = 0.0          # X position (m) in odom frame
        self.y = 0.0          # Y position (m)
        self.theta = 0.0      # Orientation (rad)
        self.vx = 0.0         # Current linear velocity (m/s)
        self.vth = 0.0        # Current angular velocity (rad/s)
        self.last_time = rospy.Time.now()

        # Variables to compute wheel rotation angles (for joint_states)
        self.left_wheel_pos = 0.0    # Accumulated left wheel angle (rad)
        self.right_wheel_pos = 0.0   # Accumulated right wheel angle (rad)

        # ====== ROS PUBLISHERS ======
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # ====== ROS SUBSCRIBER ======
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        rospy.loginfo("[fake_odom] Node initialized successfully.")
        rospy.loginfo("[fake_odom] Track width: %.3f m | Wheel radius: %.4f m",
                      self.track_width, self.wheel_radius)
        rospy.loginfo("[fake_odom] Broadcasting TF odom->base_footprint and publishing /joint_states.")

    def cmd_vel_callback(self, msg):
        """Receive velocity commands from teleop"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        """Compute odometry and broadcast TF + topics periodically"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if dt <= 0:
            return

        # ====== POSITION INTEGRATION (Dead Reckoning) ======
        # This is an open-loop calculation: assumes the robot moves exactly as commanded
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # ====== COMPUTE WHEEL ROTATION ANGLES (for joint_states) ======
        # Inverse kinematics: robot velocity -> individual wheel velocities
        v_left = self.vx - (self.vth * self.track_width / 2.0)
        v_right = self.vx + (self.vth * self.track_width / 2.0)

        # Angular velocity of wheel = linear velocity / radius
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Accumulate rotation angle
        self.left_wheel_pos += omega_left * dt
        self.right_wheel_pos += omega_right * dt

        # ====== BROADCAST TF: odom -> base_footprint ======
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            "base_footprint",    # child frame
            "odom"               # parent frame
        )

        # ====== PUBLISH TOPIC /odom ======
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

        # ====== PUBLISH TOPIC /joint_states (Resolves wheel TF errors) ======
        js = JointState()
        js.header.stamp = current_time
        js.name = ['left_wheel_base', 'right_wheel_base']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        js.velocity = [omega_left, omega_right]
        js.effort = []

        self.joint_pub.publish(js)

        self.last_time = current_time

    def run(self):
        """Main loop - runs at 50Hz"""
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
