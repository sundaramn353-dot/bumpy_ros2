
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import threading
import time
import sys

class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10)
        self.waypoints = []
        self.get_logger().info('Waypoint Collector Initialized.')
        self.get_logger().info('Use the "Publish Point" tool in RViz to add waypoints.')

    def point_callback(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1.0  # Default orientation
        
        self.waypoints.append(pose)
        self.get_logger().info(f'Added Waypoint #{len(self.waypoints)}: x={msg.point.x:.2f}, y={msg.point.y:.2f}')

def main():
    rclpy.init()
    
    # Init Nav2
    navigator = BasicNavigator()
    
    # Init Collector Node
    collector = WaypointCollector()
    
    # Spin in a separate thread to not block input
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(collector)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("-------------------------------------------------")
    print("Interactive Waypoint Follower")
    print("-------------------------------------------------")
    print("1. In RViz, use the 'Publish Point' tool (Toolbar) to click separate locations.")
    print("2. Each click adds a waypoint.")
    print("3. Press ENTER here to start following the path.")
    print("-------------------------------------------------")
    
    try:
        input("Press Enter to START navigation...\n")
    except KeyboardInterrupt:
        return

    if not collector.waypoints:
        print("No waypoints collected! Exiting.")
        return

    print(f"Sending {len(collector.waypoints)} waypoints to Nav2...")
    # navigator.lifecycleStartup() # assume already active or handled
    navigator.waitUntilNav2Active()

    navigator.followWaypoints(collector.waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and feedback.current_waypoint < len(collector.waypoints):
            print(f"Executing Waypoint {feedback.current_waypoint + 1}/{len(collector.waypoints)}", end='\r')
        time.sleep(1.0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\nGoal Succeeded!")
    elif result == TaskResult.CANCELED:
        print("\nGoal Canceled!")
    elif result == TaskResult.FAILED:
        print("\nGoal Failed!")

    # navigator.lifecycleShutdown() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
