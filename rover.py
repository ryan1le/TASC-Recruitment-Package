import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rover_utils.msg import Sensor #Custom message
from geometry_msgs.msg import Point
from std_msgs.msg import Int64
import random

class Rover(Node):
    def __init__(self):
        super().__init__('rover_node')

        self.get_logger().info("Rover node started")

        self.base_node_alive = False

        self.node_test_timer = self.create_timer(
            0.3,
            self.node_test_callback)
        
        self.rover_pose_timer = self.create_timer(
            0.2,
            self.rover_pose_callback)

        self.node_test_publisher = self.create_publisher(
            Int64,
            'node_test',
            10)
        
        self.rover_pose = self.create_publisher(
            Sensor,
            'rover_pose',
            10)
        
    #Checks to see if the base node is alive, if not rover will shut down
    def node_test_callback(self):
        self.node_alive = self.get_subscriptions_info_by_topic('node_test')

        if not self.node_alive:
            self.get_logger().error("Base Node Offline!")
            self.stop()
        elif self.node_alive and not self.base_node_alive:
            self.get_logger().info("Base Node Connected!")
            self.base_node_alive = True

    
    def rover_pose_callback(self):
        sensor_msg = Sensor()
        point_msg = Point()
        # Generate random latitude and longitude within some plausible range
        point_msg.x = random.uniform(-90, 90)      # latitude
        point_msg.y = random.uniform(-180, 180)    # longitude
        point_msg.z = random.uniform(0, 360)       # heading in degrees

        sensor_msg.coords = point_msg
        sensor_msg.linear = random.uniform(0, 5)   # linear velocity
        sensor_msg.angular = random.uniform(-3.14, 3.14) # angular

        self.rover_pose.publish(sensor_msg)

    
    def stop(self):
        self.get_logger().info("Shutting down rover node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    rover_node = Rover()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(rover_node, executor=executor)
    except:
        rclpy.shutdown()

if __name__ == '__main__':
    main()