#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import tf_transformations

try:
    from mpu6050 import mpu6050
except ImportError:
    mpu6050 = None

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Parameters
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 100.0) # Hz
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('base_frame_id', 'base_link') # For TF
        self.declare_parameter('odom_frame_id', 'odom') # Parent frame for TF
        self.declare_parameter('calibrate_on_start', True)
        self.declare_parameter('deadband', 0.05) # Reduced deadband, relying on better calib + filter
        self.declare_parameter('publish_tf', False) # Disabled by default/request
        self.declare_parameter('invert_z', False) # Invert Z axis if rotation is reversed

        # Covariance Parameters (defaults)
        self.declare_parameter('orientation_covariance', [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01])
        self.declare_parameter('angular_velocity_covariance', [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001])
        self.declare_parameter('linear_acceleration_covariance', [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001])

        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.do_calibration = self.get_parameter('calibrate_on_start').value
        self.deadband = self.get_parameter('deadband').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.invert_z = self.get_parameter('invert_z').value
        
        self.orientation_covariance = self.get_parameter('orientation_covariance').value
        self.angular_velocity_covariance = self.get_parameter('angular_velocity_covariance').value
        self.linear_acceleration_covariance = self.get_parameter('linear_acceleration_covariance').value

        # Initialize MPU6050
        if mpu6050 is None:
            self.get_logger().error("mpu6050-raspberrypi library not found. Please install it: pip install mpu6050-raspberrypi")
            return

        try:
            self.mpu = mpu6050(self.i2c_address)
            self.get_logger().info(f"Connected to MPU6050 at 0x{self.i2c_address:02x}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MPU6050: {e}")
            return

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.yaw = 0.0
        self.gz_bias = 0.0
        self.last_time = time.time()
        
        # Calibration
        if self.do_calibration:
            self.calibrate_gyro()

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def calibrate_gyro(self):
        self.get_logger().info("Starting calibration... DO NOT MOVE ROBOT.")
        # Wait a bit before starting to let vibrations settle
        time.sleep(2.0)
        
        samples = 3000
        gz_sum = 0.0
        for _ in range(samples):
            try:
                gyro_data = self.mpu.get_gyro_data()
                gz_val = gyro_data['z']
                if self.invert_z:
                    gz_val = -gz_val
                gz_sum += gz_val
            except Exception as e:
                self.get_logger().warn(f"Error reading gyro during calibration: {e}")
            time.sleep(0.001) 
        
        self.gz_bias = gz_sum / samples
        self.get_logger().info(f"Calibration complete. Bias: {self.gz_bias:.4f} deg/s")

    def timer_callback(self):
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            # Read Gyro Data
            gyro_data = self.mpu.get_gyro_data()
            gz_raw = gyro_data['z']
            
            if self.invert_z:
                gz_raw = -gz_raw
            
            # Read Accel Data
            accel_data = self.mpu.get_accel_data()
            ax = accel_data['x']
            ay = accel_data['y']
            az = accel_data['z']

            # Subtract Bias
            gz = gz_raw - self.gz_bias
            
            # Deadband Check
            if abs(gz) < self.deadband:
                gz = 0.0

            # Integrate Yaw
            self.yaw += gz * dt
            
            # Normalize Yaw to [-180, 180]
            self.yaw = (self.yaw + 180) % 360 - 180

            # Convert to radians
            yaw_rad = math.radians(self.yaw)
            
            # Create Quaternion from Yaw
            q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
            orientation = Quaternion()
            orientation.x = q[0]
            orientation.y = q[1]
            orientation.z = q[2]
            orientation.w = q[3]

            # Publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame_id
            
            # Set orientation
            imu_msg.orientation = orientation
            imu_msg.orientation_covariance = self.orientation_covariance
            
            # Set angular velocity (convert to rad/s)
            imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
            imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
            imu_msg.angular_velocity.z = math.radians(gz) # calibrated
            imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
            
            # Set linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
            
            self.imu_pub.publish(imu_msg)

            # Broadcast Transform if enabled
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.odom_frame_id
                t.child_frame_id = self.base_frame_id
                
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation = orientation
                
                self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
