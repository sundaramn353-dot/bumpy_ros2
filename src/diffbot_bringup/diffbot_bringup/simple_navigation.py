import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistStamped
import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t3 = +2.0 * (w * y - z * x)
    t3 = +1.0 if t3 > +1.0 else t3
    t3 = -1.0 if t3 < -1.0 else t3
    pitch_y = math.asin(t3)
    
    t4 = +2.0 * (w * z + x * y)
    t5 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t4, t5)
    
    return roll_x, pitch_y, yaw_z # in radians

class SimpleNavigationNode(Node):

    def __init__(self):
        super().__init__("simple_navigation_node")

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0 # Added target orientation
        self.target_received = False
        
        self.distance_tolerance = 0.1

        # Subscribers
        self.odom_subscriber_ = self.create_subscription(Odometry, '/bumpy_gamma/odom', self.callback_odometry, 10)
        self.target_subscriber_ = self.create_subscription(Pose, '/target_pose', self.callback_target, 10)
        
        # Publisher
        self.cmd_vel_publisher_ = self.create_publisher(TwistStamped, '/bumpy_gamma/cmd_vel', 10)
        
        # Control Loop
        self.timer_ = self.create_timer(0.02, self.control_loop) # 50 Hz

        self.get_logger().info("Simple Navigation Node Started. Waiting for target pose...")

    def callback_odometry(self, odom: Odometry):
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        
        # Quaternion to Euler
        q_x = odom.pose.pose.orientation.x
        q_y = odom.pose.pose.orientation.y
        q_z = odom.pose.pose.orientation.z
        q_w = odom.pose.pose.orientation.w
        
        _, _, self.current_theta = euler_from_quaternion(q_x, q_y, q_z, q_w)

    def callback_target(self, msg: Pose):
        self.target_x = msg.position.x
        self.target_y = msg.position.y
        
        # Extract target orientation
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w
        _, _, self.target_theta = euler_from_quaternion(q_x, q_y, q_z, q_w)
        self.target_received = True
        
        self.get_logger().info(f"New Target: x={self.target_x}, y={self.target_y}, th={self.target_theta:.2f}")

    def control_loop(self):
        if not self.target_received:
            return

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        # Calculate the distance
        distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)

        # 1. Check Distance (Go-To-XY Phase)
        if distance > 0.03:
            # Calculate heading error relative to target POSITION (not orientation yet)
            angle_to_target = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            heading_error = angle_to_target - self.current_theta
            
            # Normalize -PI to PI
            while heading_error > math.pi: heading_error -= 2 * math.pi
            while heading_error < -math.pi: heading_error += 2 * math.pi

            # 1. Continuous Control Logic (Smooth Arcing)
            # The user wants "U-turn" behavior, so we feed both linear and angular velocities continuously.
            
            # Constants
            K_linear = 2.0  # Lower gain for smoother approach
            K_angular = 4.0 # Reacts to heading error
            
            MAX_LINEAR_SPEED = 0.5
            MIN_LINEAR_SPEED = 0.0 # PID handles low speed
            MAX_ANGULAR_SPEED = 1.0

            # Calculate Raw Speeds
            raw_linear = K_linear * distance
            raw_angular = K_angular * heading_error
            
            # Smart Speed Reduction: Slow down if heading error is large
            # If error is 90 deg (1.57 rad), we want to slow down significantly to turn tight
            # If error is 0, we go full speed.
            # Scale linear speed by cosine of error? Or just simple clamp.
            
            # User behavior request: "linear moves... uturn kind of right"
            # This implies driving WHILE turning.
            
            # Safety: If facing completely backwards (>90 deg), maybe slow down linear a lot
            if abs(heading_error) > 1.57: # > 90 degrees
                raw_linear *= 0.1 # Crawl while turning around
            elif abs(heading_error) > 0.5: # > 30 degrees
                raw_linear *= 0.5 # Half speed while turning
            
            # Apply Clamps
            twist.twist.linear.x = max(min(raw_linear, MAX_LINEAR_SPEED), MIN_LINEAR_SPEED)
            twist.twist.angular.z = max(min(raw_angular, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)
            
            self.cmd_vel_publisher_.publish(twist)
            return



        # 3. Target Reached Completely
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_vel_publisher_.publish(twist)
        self.get_logger().info("Target Reached. Navigation Complete. Waiting for new target...")
        self.target_received = False # Disable loop to allow Teleop

def main():
    rclpy.init()
    node = SimpleNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()