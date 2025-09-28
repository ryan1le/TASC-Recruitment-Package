import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rover_utils.msg import Sensor #Custom message
from std_msgs.msg import Int64

class Base(Node):
    def __init__(self):
        super().__init__('base_node')

        self.get_logger().info("Base node started")

        self.node_test_timer = self.create_timer(
            0.3,
            self.watchdog_callback)

        self.node_sub = self.create_subscription(
            Int64,
            'node_test',
            self.node_callback,
            10)
        
        self.rover_pose_sub = self.create_subscription(
            Sensor,
            'rover_pose',
            self.rover_pose_callback,
            10)
        
    def node_callback(self, msg):
        None
    
    def watchdog_callback(self):
        self.node_alive = self.get_publishers_info_by_topic('node_test')

        if not self.node_alive:
            self.get_logger().error("Rover Node Offline!")


    def rover_pose_callback(self, msg):
        self.get_logger().info(f"Rover Position - Latitude: {msg.coords.x}, Longitude: {msg.coords.y}, Heading: {msg.coords.z}")
        self.get_logger().info(f"Rover Velocity - Linear: {msg.linear}, Angular: {msg.angular}")


def main(args=None):
    rclpy.init(args=args)

    base_node = Base()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(base_node, executor=executor)
    except:
        rclpy.shutdown()

if __name__ == '__main__':
    main()