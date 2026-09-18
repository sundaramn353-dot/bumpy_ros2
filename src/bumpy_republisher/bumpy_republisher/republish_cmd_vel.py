#!/usr/bin/env python3
'''
import rclpy
import roslibpy


from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosidl_runtime_py import set_message_fields


class RepublishCmdVel(Node):

    def __init__(self):
        super().__init__("republish_cmd_vel")

        # Declare parameters
        self.declare_parameter('websocket_host', '192.168.1.13')
        self.declare_parameter('websocket_port', 9090)

        self.host = self.get_parameter('websocket_host').get_parameter_value().string_value
        self.port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.client = None
        self.remote_cmd_vel_pub = None
        self.connected = False

        # Teleop Forwarding: Subscribe to local /cmd_vel
       # self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/bumpy_gamma/cmd_vel', self.cmd_vel_callback, 10)
        
        # Timer to check connection
        self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info(f"Initialized. Attempting to connect to {self.host}:{self.port}...")

    def check_connection(self):
        if self.client and self.client.is_connected:
            return

        self.connected = False
        try:
            self.get_logger().info(f"Connecting to Rosbridge at {self.host}...")
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()
            self.connected = True
            
            # Re-initialize publisher
            #self.remote_cmd_vel_pub = roslibpy.Topic(self.client, '/bumpy_alpha/cmd_vel', 'geometry_msgs/msg/TwistStamped')
            self.remote_cmd_vel_pub = roslibpy.Topic(self.client, '/bumpy_gamma/cmd_vel', 'geometry_msgs/msg/Twist')
            self.get_logger().info("Connected to Rosbridge! remote_cmd_vel_pub initialized.")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to connect: {e}. Retrying in 2 seconds...")
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None

    def cmd_vel_callback(self, msg):
        if not self.connected or not self.remote_cmd_vel_pub:
            return

        try:
            # Construct TwistStamped dictionary for roslibpy
            # Current time in nanoseconds
            now_ns = self.get_clock().now().nanoseconds
            secs = int(now_ns / 1e9)
            nsecs = int(now_ns % 1e9)

            twist_stamped_msg = {
                'header': {
                    'stamp': {
                        'sec': secs,
                        'nanosec': nsecs
                    },
                    'frame_id': '' 
                },
                'twist': {
                    'linear': {
                        'x': msg.linear.x,
                        'y': msg.linear.y,
                        'z': msg.linear.z
                    },
                    'angular': {
                        'x': msg.angular.x,
                        'y': msg.angular.y,
                        'z': msg.angular.z
                    }
                }
            }
            
            # Publish to websocket
            self.remote_cmd_vel_pub.publish(roslibpy.Message(twist_stamped_msg))
            
        except Exception as e:
            self.get_logger().error(f"Error forwarding cmd_vel: {e}")
    

    def destroy_node(self):
        if self.client and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()
def main():
    rclpy.init()

    cmd_vel_republsiher = RepublishCmdVel()
    
    try:
        rclpy.spin(cmd_vel_republsiher)
    except KeyboardInterrupt:
        pass
    
    # Clean shutdown
    cmd_vel_republsiher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    '''

    #!/usr/bin/env python3
import rclpy
import roslibpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class RepublishCmdVel(Node):

    def __init__(self):
        super().__init__("republish_cmd_vel")

        # Declare parameters
        self.declare_parameter('websocket_host', '192.168.1.13')
        self.declare_parameter('websocket_port', 9090)

        self.host = self.get_parameter('websocket_host').get_parameter_value().string_value
        self.port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.client = None
        self.remote_cmd_vel_pub = None
        self.connected = False

        # Subscribe to the safety-checked, smoothed cmd_vel (collision_monitor's output)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/bumpy_gamma/cmd_vel', self.cmd_vel_callback, 10)

        # Timer to check connection
        self.create_timer(2.0, self.check_connection)

        self.get_logger().info(f"Initialized. Attempting to connect to {self.host}:{self.port}...")

    def check_connection(self):
        if self.client and self.client.is_connected:
            return

        self.connected = False
        try:
            self.get_logger().info(f"Connecting to Rosbridge at {self.host}...")
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()
            self.connected = True

            # Re-initialize publisher — plain Twist, matching simple_controller's subscription
            self.remote_cmd_vel_pub = roslibpy.Topic(
                self.client, '/bumpy_gamma/cmd_vel', 'geometry_msgs/msg/Twist')
            self.get_logger().info("Connected to Rosbridge! remote_cmd_vel_pub initialized.")

        except Exception as e:
            self.get_logger().warn(f"Failed to connect: {e}. Retrying in 2 seconds...")
            if self.client:
                try:
                    self.client.close()
                except Exception:
                    pass
                self.client = None

    def cmd_vel_callback(self, msg):
        if not self.connected or not self.remote_cmd_vel_pub:
            return

        try:
            twist_msg = {
                'linear': {
                    'x': msg.linear.x,
                    'y': msg.linear.y,
                    'z': msg.linear.z
                },
                'angular': {
                    'x': msg.angular.x,
                    'y': msg.angular.y,
                    'z': msg.angular.z
                }
            }

            # Publish to websocket
            self.remote_cmd_vel_pub.publish(roslibpy.Message(twist_msg))

        except Exception as e:
            self.get_logger().error(f"Error forwarding cmd_vel: {e}")

    def destroy_node(self):
        if self.client and self.client.is_connected:
            self.client.terminate()
        super().destroy_node()


def main():
    rclpy.init()

    cmd_vel_republisher = RepublishCmdVel()

    try:
        rclpy.spin(cmd_vel_republisher)
    except KeyboardInterrupt:
        pass

    # Clean shutdown
    cmd_vel_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()