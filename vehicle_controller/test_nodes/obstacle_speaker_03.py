# Just for debugging purposes

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from my_bboxes_msg.msg import YoloObstacle, VehiclePhase

class ObstacleSpeaker(Node):
    def __init__(self):
        super().__init__('obstacle_speaker')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.phase = 0
        self.subphase = 'before flight'

        self.publisher_ = self.create_publisher(YoloObstacle, '/yolo_obstacle', qos_profile)
        self.vehicle_phase_subscriber = self.create_subscription(VehiclePhase, '/vehicle_phase', self.phase_callback, qos_profile)

        self.time_counter = 0
        self.time_counter_2 = 0

        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        if self.subphase == 'go slow':
            self.time_counter += 1

        if self.time_counter > 40:
            msg = YoloObstacle()
            msg.label = "ladder"
            msg.x = 635.0                   # left. left < 640 < right. critical section = 25px
            msg.y = 300.0
            self.publisher_.publish(msg)

    def phase_callback(self, msg):
        # get vehicle phase
        self.phase = msg.phase
        self.subphase = msg.subphase

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpeaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
