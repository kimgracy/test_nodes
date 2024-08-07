# Just for debugging purposes

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from my_bboxes_msg.msg import YoloObstacle

class ObstacleSpeaker(Node):
    def __init__(self):
        super().__init__('obstacle_speaker')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter('x', 0.5)
        self.declare_parameter('y', 0.5)

        self.publisher_ = self.create_publisher(YoloObstacle, '/yolo_obstacle', qos_profile)
        self.timer = self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        msg = YoloObstacle()
        msg.label = "ladder"
        msg.x = self.get_parameter('x').get_parameter_value().double_value
        msg.y = self.get_parameter('y').get_parameter_value().double_value
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpeaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()