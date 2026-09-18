'''
import rclpy
from rclpy.node import Node
import roslibpy
from sensor_msgs.msg import LaserScan

class RepublishScan(Node):
    def __init__(self):
        super().__init__("republish_scan")
        
        # Declare parameters
        self.declare_parameter('websocket_host', '192.168.1.13')
        self.declare_parameter('websocket_port', 9090)
        
        host = self.get_parameter('websocket_host').get_parameter_value().string_value
        port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        
        self.get_logger().info(f"Connecting to websocket at {host}:{port}")

        # Initialize ROS 2 Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

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
            self.listener = roslibpy.Topic(self.client, '/scan', 'sensor_msgs/LaserScan')
            self.listener.subscribe(self.scan_callback)
            self.get_logger().info("Subscribed to remote topic /scan")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to websocket: {e}. Retrying in 2 seconds...")
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None

    def scan_callback(self, msg):
        try:
            scan_msg = LaserScan()
            
            # Header
            now = self.get_clock().now()
            # Log difference between msg stamp (if available) and now
            # Note: msg['header']['stamp'] is a dict {sec, nanosec}
            if 'header' in msg and 'stamp' in msg['header']:
                msg_sec = msg['header']['stamp']['sec']
                now_sec = now.seconds_nanoseconds()[0]
                diff = msg_sec - now_sec
                # self.get_logger().info(f"Scan Skew: {diff}s (Msg: {msg_sec}, Now: {now_sec})")

            scan_msg.header.stamp = now.to_msg()
            # scan_msg.header.frame_id = msg['header']['frame_id']
            scan_msg.header.frame_id = 'bumpy_gamma/laser_frame'  # Force frame_id to match URDF
            
            scan_msg.angle_min = float(msg['angle_min'])
            scan_msg.angle_max = float(msg['angle_max'])
            scan_msg.angle_increment = float(msg['angle_increment'])
            scan_msg.time_increment = float(msg['time_increment'])
            scan_msg.scan_time = float(msg['scan_time'])
            scan_msg.range_min = float(msg['range_min'])
            scan_msg.range_max = float(msg['range_max'])
            
            # Ranges and Intensities
            # Note: Depending on serialization, these might be lists or special encodings.
            # Roslibpy usually handles decoding if it's standard JSON.
            # If using binary compression (CBOR/Rosbridge features), roslibpy might need handling, 
            # but usually it's transparent or a list.
            
            scan_msg.ranges = [float(x) for x in msg['ranges']]
            
            if 'intensities' in msg:
                 scan_msg.intensities = [float(x) for x in msg['intensities']]
            
            self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing scan message: {e}")

    def destroy_node(self):
        if hasattr(self, 'client') and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    republish_scan = RepublishScan()
    
    try:
        rclpy.spin(republish_scan)
    except KeyboardInterrupt:
        pass
    finally:
        republish_scan.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''




import rclpy
from rclpy.node import Node
import roslibpy
from sensor_msgs.msg import LaserScan


class RepublishScan(Node):
    def __init__(self):
        super().__init__("republish_scan")

        # Declare parameters
        self.declare_parameter('websocket_host', '192.168.1.13')
        self.declare_parameter('websocket_port', 9090)

        host = self.get_parameter('websocket_host').get_parameter_value().string_value
        port = self.get_parameter('websocket_port').get_parameter_value().integer_value

        self.get_logger().info(f"Connecting to websocket at {host}:{port}")

        # Initialize ROS 2 Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

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
            self.listener = roslibpy.Topic(self.client, '/scan', 'sensor_msgs/LaserScan')
            self.listener.subscribe(self.scan_callback)
            self.get_logger().info("Subscribed to remote topic /scan")

        except Exception as e:
            self.get_logger().warn(f"Failed to connect to websocket: {e}. Retrying in 2 seconds...")
            if self.client:
                try:
                    self.client.close()
                except Exception:
                    pass
                self.client = None

    def scan_callback(self, msg):
        try:
            scan_msg = LaserScan()

            # Header — preserve the ROBOT's original capture timestamp instead of
            # the offboard machine's local receive time. Re-stamping with
            # self.get_clock().now() made this scan's effective timing drift
            # independently from the bridged odom TF (different, jittering network
            # delays), so the costmap paired laser points with a pose the robot
            # wasn't actually in yet. Requires clocks on both machines to be
            # synchronized (chrony/NTP) for these stamps to be meaningful here.
            scan_msg.header.stamp.sec = int(msg['header']['stamp']['sec'])
            scan_msg.header.stamp.nanosec = int(msg['header']['stamp']['nanosec'])
            scan_msg.header.frame_id = 'bumpy_gamma/laser_frame'  # Force frame_id to match URDF

            scan_msg.angle_min = float(msg['angle_min'])
            scan_msg.angle_max = float(msg['angle_max'])
            scan_msg.angle_increment = float(msg['angle_increment'])
            scan_msg.time_increment = float(msg['time_increment'])
            scan_msg.scan_time = float(msg['scan_time'])
            scan_msg.range_min = float(msg['range_min'])
            scan_msg.range_max = float(msg['range_max'])

            scan_msg.ranges = [float(x) for x in msg['ranges']]

            if 'intensities' in msg:
                scan_msg.intensities = [float(x) for x in msg['intensities']]

            self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing scan message: {e}")

    def destroy_node(self):
        if hasattr(self, 'client') and self.client and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    republish_scan = RepublishScan()

    try:
        rclpy.spin(republish_scan)
    except KeyboardInterrupt:
        pass
    finally:
        republish_scan.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
