#!/usr/bin/env python3
"""
simple_controller.py
──────────────────────────────────────────────────────────────────────────────
Subscribes to:
  • /joint_states          – encoder positions/velocities from diffbot_gpio_node
  • /<ns>/cmd_vel          – geometry_msgs/Twist  (from Nav2 / teleop)
  • /<ns>/imu/data         – IMU for absolute yaw fusion
  • /<ns>/simple_controller/reset_odom – Bool reset trigger

Publishes:
  • /<ns>/odom             – nav_msgs/Odometry  (pose + covariance)
  • /<ns>/simple_velocity_controller/commands – Float64MultiArray [left, right] rad/s
  • TF  odom → base_footprint  (if enable_odom_tf=True)

Odometry model (differential drive):
  d_left  = wheel_radius × Δθ_left   [m]
  d_right = wheel_radius × Δθ_right  [m]
  d_s     = (d_left + d_right) / 2   [m]   – arc length
  d_theta = (d_right - d_left) / wheel_separation  [rad]  (encoder-only heading)
  x      += d_s × cos(θ)
  y      += d_s × sin(θ)
  When IMU is enabled the heading θ comes directly from IMU yaw.

N20 motor / Bumpy Gamma physical parameters
  wheel_radius     = 0.022 m   (44 mm diameter)
  wheel_separation = 0.080 m   (8 cm centre-to-centre)

TELEOP NOTE: teleop_twist_keyboard publishes to /cmd_vel (global).
  A topic_tools/relay node in bumpy_bringup relays /cmd_vel → /bumpy_gamma/cmd_vel.
  Nav2 Goal cmd_vel flow: nav2 → velocity_smoother → collision_monitor → /bumpy_gamma/cmd_vel
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64, Float64MultiArray, Bool
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("wheel_radius",          0.022)
        self.declare_parameter("wheel_separation",      0.080)
        self.declare_parameter("left_wheel_name",       "left_wheel_joint")
        self.declare_parameter("right_wheel_name",      "right_wheel_joint")
        self.declare_parameter("robot_namespace",       "")
        self.declare_parameter("enable_odom_tf",        True)
        self.declare_parameter("use_imu_for_angular_z", True)

        self.wheel_radius_     = self.get_parameter("wheel_radius").value
        self.wheel_separation_ = self.get_parameter("wheel_separation").value
        self.left_wheel_name   = self.get_parameter("left_wheel_name").value
        self.right_wheel_name  = self.get_parameter("right_wheel_name").value
        self.enable_odom_tf_   = self.get_parameter("enable_odom_tf").value
        self.use_imu_          = self.get_parameter("use_imu_for_angular_z").value

        # Resolve namespace
        param_ns = self.get_parameter("robot_namespace").value.strip('/')
        node_ns  = self.get_namespace().strip('/')
        self.robot_ns_ = param_ns or node_ns or ""

        self.get_logger().info(
            f"SimpleController  ns='{self.robot_ns_}'  "
            f"R={self.wheel_radius_:.4f} m  "
            f"Sep={self.wheel_separation_:.4f} m  "
            f"IMU={'yes' if self.use_imu_ else 'no'}"
        )

        # ── Odometry state ──────────────────────────────────────────────────
        self.x_     = 0.0
        self.y_     = 0.0
        self.theta_ = 0.0

        self.left_prev_pos_  = 0.0
        self.right_prev_pos_ = 0.0
        self.prev_time_      = self.get_clock().now()

        # Cache joint indices
        self.l_idx = None
        self.r_idx = None

        # Current velocity targets
        self.target_left_vel_  = 0.0
        self.target_right_vel_ = 0.0


        # ── Diff-drive speed-conversion matrix ─────────────────────────────
        self._update_conversion()

        # ── ROS interfaces ──────────────────────────────────────────────────
        # Wheel velocity commands → gpio node
        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray, "simple_velocity_controller/commands", 10)

        # cmd_vel (relative topic – resolves to /<robot_ns>/cmd_vel when namespaced)
        self.vel_sub_ = self.create_subscription(
            Twist, "cmd_vel", self._vel_cb, 10)

        # Joint states from gpio node (relative → in same namespace)
        self.joint_sub_ = self.create_subscription(
            JointState, "joint_states", self._joint_cb, qos_profile_sensor_data)

        # Odom publisher
        self.odom_pub_ = self.create_publisher(Odometry, "odom", 10)

        # Reset odom
        self.create_subscription(
            Bool, "simple_controller/reset_odom", self._reset_odom_cb, 10)

        # IMU
        self.create_subscription(
            Imu, "imu/data", self._imu_cb, qos_profile_sensor_data)


        # ── Odometry message template ───────────────────────────────────────
        pfx = f"{self.robot_ns_}/" if self.robot_ns_ else ""
        self._odom_frame      = f"{pfx}odom"
        self._base_frame      = f"{pfx}base_footprint"

        self.odom_msg_              = Odometry()
        self.odom_msg_.header.frame_id  = self._odom_frame
        self.odom_msg_.child_frame_id   = self._base_frame
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Pose covariance (diagonal): [x, y, z, roll, pitch, yaw]
        # Small values = high confidence.  z/roll/pitch are not observable.
        self.odom_msg_.pose.covariance = [
            1e-4, 0, 0, 0, 0, 0,
            0, 1e-4, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-3,
        ]
        # Twist covariance
        self.odom_msg_.twist.covariance = [
            1e-4, 0, 0, 0, 0, 0,
            0, 1e6, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-3,
        ]

        # ── TF broadcaster ──────────────────────────────────────────────────
        self.tf_br_   = TransformBroadcaster(self)
        self.tf_msg_  = TransformStamped()
        self.tf_msg_.header.frame_id = self._odom_frame
        self.tf_msg_.child_frame_id  = self._base_frame

        self.add_on_set_parameters_callback(self._param_cb)

        self.get_logger().info(
            f"TF: {self._odom_frame} → {self._base_frame}")

    # ── Parameter live-update ───────────────────────────────────────────────
    def _param_cb(self, params):
        for p in params:
            if p.name == "wheel_radius" and p.type_ == p.Type.DOUBLE:
                self.wheel_radius_ = p.value
                self._update_conversion()
            elif p.name == "wheel_separation" and p.type_ == p.Type.DOUBLE:
                self.wheel_separation_ = p.value
                self._update_conversion()
        return SetParametersResult(successful=True)

    def _update_conversion(self):
        """Rebuild the 2×2 forward-kinematics speed-conversion matrix."""
        r = self.wheel_radius_
        b = self.wheel_separation_
        self.speed_conversion_ = np.array([
            [r / 2,  r / 2],
            [-r / b, r / b],
        ])

    # ── IMU callback ────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        if not self.use_imu_:
            return
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.theta_ = yaw

    # ── cmd_vel callback ────────────────────────────────────────────────────
    def _vel_cb(self, msg: Twist):
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        wheel_speed = np.linalg.solve(self.speed_conversion_, robot_speed)
        self.target_left_vel_  = float(wheel_speed[0, 0])
        self.target_right_vel_ = float(wheel_speed[1, 0])

    # ── JointState callback (main odometry update) ───────────────────────────
    def _joint_cb(self, msg: JointState):
        # Resolve joint indices on first call
        if self.l_idx is None or self.r_idx is None:
            try:
                self.l_idx = msg.name.index(self.left_wheel_name)
                self.r_idx = msg.name.index(self.right_wheel_name)
            except ValueError:
                return

        l = self.l_idx
        r = self.r_idx

        # ── Δt  ─────────────────────────────────────────────────────────────
        # Use the node clock (not msg.header.stamp) to avoid stale-stamp issues
        curr_time = self.get_clock().now()
        dt = (curr_time - self.prev_time_).nanoseconds / S_TO_NS

        if dt <= 1e-6:
            return

        # ── Δθ wheel (radians) ───────────────────────────────────────────────
        # NOTE: diffbot_gpio now publishes forward = POSITIVE ticks directly
        # (the sign correction lives at the source in diffbot_gpio.cpp), so
        # no negation is needed here anymore. Previously this file negated
        # both deltas to correct for forward=negative raw ticks — but that
        # correction was local to this file's odometry math only, and was
        # never applied to the wheel-velocity *targets* sent the other way
        # to diffbot_gpio, which caused diffbot_gpio's PID feedback loop to
        # compare targets and measured velocity in opposite sign
        # conventions. Fixing the convention at the source (diffbot_gpio.cpp)
        # instead keeps target and feedback consistent everywhere.
        dp_left  = msg.position[l] - self.left_prev_pos_
        dp_right = msg.position[r] - self.right_prev_pos_

        # Guard against wrap-around or initial jump
        if abs(dp_left) > 6.0 * math.pi or abs(dp_right) > 6.0 * math.pi:
            # Reset reference without updating pose
            self.left_prev_pos_  = msg.position[l]
            self.right_prev_pos_ = msg.position[r]
            self.prev_time_      = curr_time
            return

        self.left_prev_pos_  = msg.position[l]
        self.right_prev_pos_ = msg.position[r]
        self.prev_time_      = curr_time

        # ── Arc displacement [m] ─────────────────────────────────────────────
        d_left  = self.wheel_radius_ * dp_left
        d_right = self.wheel_radius_ * dp_right
        d_s     = (d_left + d_right) / 2.0   # signed arc length

        # ── Heading update ───────────────────────────────────────────────────
        if not self.use_imu_:
            d_theta_enc = (d_right - d_left) / self.wheel_separation_
            self.theta_ += d_theta_enc
            self.theta_  = math.atan2(math.sin(self.theta_), math.cos(self.theta_))
        # else: theta_ is kept up-to-date by _imu_cb

        # ── Position update ──────────────────────────────────────────────────
        if abs(d_s) > 1e-6:
            self.x_ += d_s * math.cos(self.theta_)
            self.y_ += d_s * math.sin(self.theta_)

        # ── Velocities for twist ─────────────────────────────────────────────
        linear  = d_s / dt
        angular = ((d_right - d_left) / self.wheel_separation_) / dt

        # ── Publish odom + TF ────────────────────────────────────────────────
        self._publish_odom(curr_time, linear, angular)

        # ── Publish wheel velocity commands ─────────────────────────────────
        cmd_msg      = Float64MultiArray()
        cmd_msg.data = [self.target_left_vel_, self.target_right_vel_]
        self.wheel_cmd_pub_.publish(cmd_msg)

    # ── Reset odom ──────────────────────────────────────────────────────────
    def _reset_odom_cb(self, msg: Bool):
        if msg.data:
            self.x_ = self.y_ = self.theta_ = 0.0
            self.get_logger().info("Odometry reset to origin.")

    # ── Publish odom message + optional TF ─────────────────────────────────
    def _publish_odom(self, stamp_rclpy_time, linear: float, angular: float):
        q   = quaternion_from_euler(0.0, 0.0, self.theta_)
        now = stamp_rclpy_time.to_msg()

        msg = self.odom_msg_
        msg.header.stamp                   = now
        msg.pose.pose.position.x           = self.x_
        msg.pose.pose.position.y           = self.y_
        msg.pose.pose.orientation.x        = q[0]
        msg.pose.pose.orientation.y        = q[1]
        msg.pose.pose.orientation.z        = q[2]
        msg.pose.pose.orientation.w        = q[3]
        msg.twist.twist.linear.x           = linear
        msg.twist.twist.angular.z          = angular
        self.odom_pub_.publish(msg)

        if self.enable_odom_tf_:
            tf              = self.tf_msg_
            tf.header.stamp = now
            tf.transform.translation.x = self.x_
            tf.transform.translation.y = self.y_
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x    = q[0]
            tf.transform.rotation.y    = q[1]
            tf.transform.rotation.z    = q[2]
            tf.transform.rotation.w    = q[3]
            self.tf_br_.sendTransform(tf)


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()