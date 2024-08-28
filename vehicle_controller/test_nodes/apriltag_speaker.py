# Just for debugging purposes

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray

import numpy as np

class ApriltagSpeaker(Node):
    def __init__(self):
        super().__init__('apriltag_speaker')
        self.home_position = np.array([-0.00599505, -0.02501676,  0.56378174])
        self.publisher_ = self.create_publisher(Float32MultiArray, 'bezier_waypoint/raw', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
    
    def timer_callback(self):
        msg = Float32MultiArray()
        tag_world = self.home_position + np.random.normal(0, 0.1, 3)              # Add noise to the home position
        msg.data = [tag_world[0], tag_world[1], tag_world[2] + 0.4, 0., 0., 0.5]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ApriltagSpeaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
