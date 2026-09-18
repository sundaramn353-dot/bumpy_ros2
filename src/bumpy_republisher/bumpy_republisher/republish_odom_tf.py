'''

import rclpy
from rclpy.node import Node
import roslibpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Twist, Vector3
import tf2_ros
import threading

class RepublishOdomTf(Node):

    def __init__(self):
        super().__init__("republish_odom_tf")
        
        # Declare parameters
        self.declare_parameter('websocket_host', '192.168.1.13')
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('publish_tf', True)
        
        host = self.get_parameter('websocket_host').get_parameter_value().string_value
        port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        self.get_logger().info(f"Connecting to websocket at {host}:{port}")

        # Initialize ROS 2 Publisher and TF Broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)



        # Initialize Roslibpy Client
        self.client = None
        self.listener = None
        
        # Timer for connection management
        self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info(f"Initialized. Attempting to connect to {host}:{port}")

    def check_connection(self):
        if self.client and self.client.is_connected:
            return

        try:
            host = self.get_parameter('websocket_host').get_parameter_value().string_value
            port = self.get_parameter('websocket_port').get_parameter_value().integer_value
            
            self.get_logger().info(f"Connecting to websocket at {host}:{port}")
            self.client = roslibpy.Ros(host=host, port=port)
            self.client.run()
            self.get_logger().info("Connected to websocket server")
            
            # Subscribe to remote topic
            # self.listener = roslibpy.Topic(self.client, '/bumpy_controller/odom', 'nav_msgs/Odometry')
            self.listener = roslibpy.Topic(self.client, '/bumpy_gamma/odom', 'nav_msgs/Odometry')
            self.listener.subscribe(self.odom_callback)
            self.get_logger().info("Subscribed to remote topic")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to websocket: {e}. Retrying in 2 seconds...")
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None

    def odom_callback(self, msg):
        try:
            # Create Odometry message
            odom_msg = Odometry()
            
            # Header
            # self.get_logger().info(f"Received odom msg from {msg['header']['frame_id']}")
            # We use the current local time for TF to be valid in RViz
            # Alternatively, we could use msg['header']['stamp'] if clocks are synchronized
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            # odom_msg.header.frame_id = msg['header']['frame_id']
            odom_msg.header.frame_id = 'bumpy_gamma/odom'
            odom_msg.child_frame_id = 'bumpy_gamma/base_footprint'  # Force consistency with URDF
            # odom_msg.child_frame_id = 'base_link' # Force consistency with URDF
            
            # ... (lines 83-138 omitted for brevity in thought, but must be preserved or handled. 
            # Wait, replace_file_content requires exact match. I should target smaller chunks.
            
            # I will split this into two edits if needed, or target the specific block.
            # Target 1: Force child_frame_id
            # Target 2: enhanced logging in TF block
            
            # Actually, I'll just do the child_frame_id assignment first.


            # Pose
            odom_msg.pose.pose.position = Point(
                x=msg['pose']['pose']['position']['x'],
                y=msg['pose']['pose']['position']['y'],
                z=msg['pose']['pose']['position']['z']
            )
            odom_msg.pose.pose.orientation = Quaternion(
                x=msg['pose']['pose']['orientation']['x'],
                y=msg['pose']['pose']['orientation']['y'],
                z=msg['pose']['pose']['orientation']['z'],
                w=msg['pose']['pose']['orientation']['w']
            )

            # Twist
            odom_msg.twist.twist.linear = Vector3(
                x=msg['twist']['twist']['linear']['x'],
                y=msg['twist']['twist']['linear']['y'],
                z=msg['twist']['twist']['linear']['z']
            )
            odom_msg.twist.twist.angular = Vector3(
                x=msg['twist']['twist']['angular']['x'],
                y=msg['twist']['twist']['angular']['y'],
                z=msg['twist']['twist']['angular']['z']
            )

            # Covariance (Required for EKF)
            # msg['pose']['covariance'] is a list of 36 floats
            if 'covariance' in msg['pose'] and len(msg['pose']['covariance']) == 36:
                odom_msg.pose.covariance = msg['pose']['covariance']
            else:
                # Default covariance if missing (adjust values as needed)
                # x, y, z, roll, pitch, yaw
                odom_msg.pose.covariance = [
                    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01
                ]

            if 'covariance' in msg['twist'] and len(msg['twist']['covariance']) == 36:
                odom_msg.twist.covariance = msg['twist']['covariance']
            else:
                 # Default covariance
                odom_msg.twist.covariance = [
                    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01
                ]

            # Publish Odom
            self.odom_pub.publish(odom_msg)

            # Broadcast TF
            # Only if enabled (prevents conflict with EKF if running)
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = odom_msg.header.frame_id
                t.child_frame_id = odom_msg.child_frame_id
     
                t.transform.translation.x = odom_msg.pose.pose.position.x
                t.transform.translation.y = odom_msg.pose.pose.position.y
                t.transform.translation.z = odom_msg.pose.pose.position.z
                t.transform.rotation = odom_msg.pose.pose.orientation
     
                self.tf_broadcaster.sendTransform(t)
                # self.get_logger().info(f"Published TF {t.header.frame_id}->{t.child_frame_id} at {t.header.stamp.sec}.{t.header.stamp.nanosec}")

        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def destroy_node(self):
        if hasattr(self, 'client') and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    republish_odom_tf = RepublishOdomTf()
    
    try:
        rclpy.spin(republish_odom_tf)
    except KeyboardInterrupt:
        pass
    finally:
        republish_odom_tf.destroy_node()
        rclpy.shutdown()
'''


import rclpy
from rclpy.node import Node
import roslibpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Vector3
import tf2_ros


class RepublishOdomTf(Node):

    def __init__(self):
        super().__init__("republish_odom_tf")

        # Declare parameters
        self.declare_parameter('websocket_host', '192.168.1.13')
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('publish_tf', True)

        host = self.get_parameter('websocket_host').get_parameter_value().string_value
        port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self.get_logger().info(f"Connecting to websocket at {host}:{port}")

        # Initialize ROS 2 Publisher and TF Broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize Roslibpy Client
        self.client = None
        self.listener = None

        # Timer for connection management
        self.create_timer(2.0, self.check_connection)

        self.get_logger().info(f"Initialized. Attempting to connect to {host}:{port}")

    def check_connection(self):
        if self.client and self.client.is_connected:
            return

        try:
            host = self.get_parameter('websocket_host').get_parameter_value().string_value
            port = self.get_parameter('websocket_port').get_parameter_value().integer_value

            self.get_logger().info(f"Connecting to websocket at {host}:{port}")
            self.client = roslibpy.Ros(host=host, port=port)
            self.client.run()
            self.get_logger().info("Connected to websocket server")

            # Subscribe to remote topic
            self.listener = roslibpy.Topic(self.client, '/bumpy_gamma/odom', 'nav_msgs/Odometry')
            self.listener.subscribe(self.odom_callback)
            self.get_logger().info("Subscribed to remote topic")

        except Exception as e:
            self.get_logger().warn(f"Failed to connect to websocket: {e}. Retrying in 2 seconds...")
            if self.client:
                try:
                    self.client.close()
                except Exception:
                    pass
                self.client = None

    def odom_callback(self, msg):
        try:
            # Create Odometry message
            odom_msg = Odometry()

            # Header — preserve the ROBOT's original capture timestamp.
            # Re-stamping with local receive time (self.get_clock().now()) makes
            # this TF's timing drift independently from the bridged /scan TF,
            # which corrupts scan-to-pose association in the costmap.
            # Requires the robot Pi and this machine to have synchronized clocks
            # (run chrony/NTP on both) — otherwise transforms will look "too old"
            # or "in the future" and get rejected/extrapolated incorrectly.
            odom_msg.header.stamp.sec = int(msg['header']['stamp']['sec'])
            odom_msg.header.stamp.nanosec = int(msg['header']['stamp']['nanosec'])
            odom_msg.header.frame_id = 'bumpy_gamma/odom'
            odom_msg.child_frame_id = 'bumpy_gamma/base_footprint'  # Force consistency with URDF

            # Pose
            odom_msg.pose.pose.position = Point(
                x=msg['pose']['pose']['position']['x'],
                y=msg['pose']['pose']['position']['y'],
                z=msg['pose']['pose']['position']['z']
            )
            odom_msg.pose.pose.orientation = Quaternion(
                x=msg['pose']['pose']['orientation']['x'],
                y=msg['pose']['pose']['orientation']['y'],
                z=msg['pose']['pose']['orientation']['z'],
                w=msg['pose']['pose']['orientation']['w']
            )

            # Twist
            odom_msg.twist.twist.linear = Vector3(
                x=msg['twist']['twist']['linear']['x'],
                y=msg['twist']['twist']['linear']['y'],
                z=msg['twist']['twist']['linear']['z']
            )
            odom_msg.twist.twist.angular = Vector3(
                x=msg['twist']['twist']['angular']['x'],
                y=msg['twist']['twist']['angular']['y'],
                z=msg['twist']['twist']['angular']['z']
            )

            # Covariance (Required for EKF / AMCL)
            if 'covariance' in msg['pose'] and len(msg['pose']['covariance']) == 36:
                odom_msg.pose.covariance = msg['pose']['covariance']
            else:
                odom_msg.pose.covariance = [
                    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01
                ]

            if 'covariance' in msg['twist'] and len(msg['twist']['covariance']) == 36:
                odom_msg.twist.covariance = msg['twist']['covariance']
            else:
                odom_msg.twist.covariance = [
                    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01
                ]

            # Publish Odom
            self.odom_pub.publish(odom_msg)

            # Broadcast TF
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = odom_msg.header.stamp
                t.header.frame_id = odom_msg.header.frame_id
                t.child_frame_id = odom_msg.child_frame_id

                t.transform.translation.x = odom_msg.pose.pose.position.x
                t.transform.translation.y = odom_msg.pose.pose.position.y
                t.transform.translation.z = odom_msg.pose.pose.position.z
                t.transform.rotation = odom_msg.pose.pose.orientation

                self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def destroy_node(self):
        if hasattr(self, 'client') and self.client and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    republish_odom_tf = RepublishOdomTf()

    try:
        rclpy.spin(republish_odom_tf)
    except KeyboardInterrupt:
        pass
    finally:
        republish_odom_tf.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
