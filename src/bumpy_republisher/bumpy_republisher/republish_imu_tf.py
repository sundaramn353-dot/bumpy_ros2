#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import roslibpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import tf2_ros

class ImuRepublisher(Node):
    def __init__(self):
        super().__init__("imu_republisher")
        
        # Declare parameters for easy connection
        self.declare_parameter('websocket_host', '192.168.1.13') # Change to your Pi's IP
        self.declare_parameter('websocket_port', 9090)
        
        host = self.get_parameter('websocket_host').get_parameter_value().string_value
        port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        
        self.get_logger().info(f"Connecting to IMU websocket at {host}:{port}")

        # Local ROS 2 Publisher and TF Broadcaster
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize Roslibpy Client
        try:
            self.client = roslibpy.Ros(host=host, port=port)
            self.client.run()
            self.get_logger().info("Connected to websocket server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        # Subscribe to remote IMU topic
        # Ensure the type 'sensor_msgs/Imu' matches exactly what is on the Pi
        self.listener = roslibpy.Topic(self.client, '/imu/data', 'sensor_msgs/Imu')
        self.listener.subscribe(self.imu_callback)

    def imu_callback(self, msg):
        try:
            imu_msg = Imu()
            
            # Use local time for header to prevent TF synchronization issues in RViz
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = msg['header'].get('frame_id', 'imu_link')

            # 1. Orientation (Quaternions)
            imu_msg.orientation.x = float(msg['orientation']['x'])
            imu_msg.orientation.y = float(msg['orientation']['y'])
            imu_msg.orientation.z = float(msg['orientation']['z'])
            imu_msg.orientation.w = float(msg['orientation']['w'])

            # 2. Angular Velocity
            imu_msg.angular_velocity.x = float(msg['angular_velocity']['x'])
            imu_msg.angular_velocity.y = float(msg['angular_velocity']['y'])
            imu_msg.angular_velocity.z = float(msg['angular_velocity']['z'])

            # 3. Linear Acceleration
            imu_msg.linear_acceleration.x = float(msg['linear_acceleration']['x'])
            imu_msg.linear_acceleration.y = float(msg['linear_acceleration']['y'])
            imu_msg.linear_acceleration.z = float(msg['linear_acceleration']['z'])

            # Publish the local ROS 2 message
            self.imu_pub.publish(imu_msg)

            # 4. Broadcast TF so RViz can visualize the rotation
            t = TransformStamped()
            t.header.stamp = imu_msg.header.stamp
            t.header.frame_id = "base_link" # Parent frame
            t.child_frame_id = imu_msg.header.frame_id # Usually imu_link
            
            # Static position (adjust if IMU isn't at the center of the robot)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation = imu_msg.orientation

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error processing IMU data: {e}")

    def destroy_node(self):
        if hasattr(self, 'client') and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()