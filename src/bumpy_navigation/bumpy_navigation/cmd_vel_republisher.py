import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRepublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_republisher')

        # Subscribe to Nav2 cmd_vel (Twist)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # Publish stamped velocity
        self.publisher = self.create_publisher(
            TwistStamped,
            '/bumpy_gamma/cmd_vel',
            10
        )

        self.get_logger().info("CmdVel Republisher Started")

    def listener_callback(self, msg: Twist):
        stamped_msg = TwistStamped()

        # Add timestamp
        stamped_msg.header.stamp = self.get_clock().now().to_msg()

        # Optional: set frame_id
        stamped_msg.header.frame_id = ""

        # Copy twist data
        stamped_msg.twist = msg

        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
