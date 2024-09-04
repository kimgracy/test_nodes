import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import deque
import numpy as np

class Filter(Node):

    def __init__(self):
        super().__init__('filter_2')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bezier_waypoint/raw',
            self.tag_callback,
            10
        )
        self.phase_sub = self.create_subscription(
            Float32MultiArray,
            '/auto_land_home_info',
            self.phase_check_callback,
            10
        )

        self.phase = 0
        self.first = 1
        self.alpha = 0.8
        self.hz_control = 1
        self.time_period = 3.0
        self.detected = False
        self.tag_world = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
        self.past_value = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
        self.publisher = self.create_publisher(Float32MultiArray, 'bezier_waypoint', 10)
        self.raw_values = deque([], maxlen=self.hz_control)  # 최대 크기를 지정하여 자동으로 관리
        
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)
        
    def main_timer_callback(self):
        if self.detected:
            avg_msg = Float32MultiArray()
            avg_msg.data = self.past_value.tolist()
            self.publisher.publish(avg_msg)
        

    def phase_check_callback(self, msg):
        print("ok")
        self.phase = 1

    def tag_callback(self, msg):
        if self.phase:
            print("tag")
            self.detected = True
            self.tag_world = np.array(msg.data)
            self.raw_values.append(self.tag_world)

            if len(self.raw_values) == self.hz_control:
                if self.first:
                    self.past_value = np.mean(self.raw_values, axis=0)
                    self.first = 0
                average = np.mean(self.raw_values, axis=0) 
                average = self.alpha * average + (1 - self.alpha) * self.past_value
                self.past_value = average

                


        
        

def main(args=None):
    rclpy.init(args=args)
    filter = Filter()
    rclpy.spin(filter)

    sub.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
