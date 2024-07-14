import os
import rclpy
import sys

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from my_bboxes_msg.msg import YoloObstacle, YoloTarget, VehiclePhase

"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint


import cv2
import torch
from cv_bridge import CvBridge
import time
import numpy as np


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # define model path and load the model
        model_path = os.path.join(os.getcwd(), '/home/gracekim/yolov5/yolov5/runs/train/exp4/weights/best.pt')
        self.model = torch.hub.load(os.path.expanduser('/home/gracekim/yolov5/yolov5'), 'custom', path=model_path, source='local')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(type(self.model))

        # create publishers
        self.publisher_obstacle = self.create_publisher(YoloObstacle, '/yolo_obstacle', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )

        # create subscribers
        self.subscriber_phase = self.create_subscription(VehiclePhase, '/vehicle_phase', self.phase_callback, qos_profile)

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        # create a subscriber for the v4l2 image topic
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # vehicle status & local position
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()        
        self.home_position = np.array([0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0])

        # create phase
        self.phase = -1

        # initial box coordinates
        self.label = 'unknown'
        self.x = 0
        self.y = 0

        # create cv_bridge instance
        self.bridge = CvBridge()
        
        # create target_capture folder, which is used to save target images
        self.target_capture_folder = os.path.join(os.getcwd(), 'src/yolo_detection/config/target_capture')
        os.makedirs(self.target_capture_folder, exist_ok=True)
        
        # timer for publishing target image
        self.timer_period = 1.0  # seconds
        self.last_capture_time = time.time()

        # timer setup
        self.offboard_heartbeat = self.create_timer(0.1, self.offboard_heartbeat_callback)
        self.takeoff_timer = self.create_timer(0.5, self.takeoff_and_arm_callback)
        self.main_timer = self.create_timer(0.5, self.main_timer_callback)
        
        print("Successfully executed: yolo_obstacle")
        print("Please switch to offboard mode.")

    
    
    "services"
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.phase = -2


    """
    Callback functions for the timers
    """
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True)

    def takeoff_and_arm_callback(self):
        if self.phase == -1 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print("Takeoff and arm")
            # takeoff and arm only if the vehicle is in offboard mode by RC switch
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.home_position = self.pos # set home position
            self.phase = 0
    
    def main_timer_callback(self):
        if self.phase == 0:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                    param1=1.0, # main mode
                    param2=6.0  # offboard
                )
                self.phase = 0.5
        elif self.phase == 0.5:
            # ladder detection
            if (YoloObstacle.label == 'ladder') :
                self.phase = 1
                print(self.phase)
                print(YoloObstacle.label)
                self.land()

    """
    Callback functions for subscribers.
    """        
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.pos = np.array([msg.x, msg.y, msg.z])
        if self.phase != -1:
            # set position relative to the home position after takeoff
            self.pos = self.pos - self.home_position

        
    """
    Functions for publishing topics.
    """
    def publish_vehicle_command(self, command, **kwargs):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", float('nan'))
        msg.param2 = kwargs.get("param2", float('nan'))
        msg.param3 = kwargs.get("param3", float('nan'))
        msg.param4 = kwargs.get("param4", float('nan'))
        msg.param5 = kwargs.get("param5", float('nan'))
        msg.param6 = kwargs.get("param6", float('nan'))
        msg.param7 = kwargs.get("param7", float('nan'))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def publish_offboard_control_mode(self, **kwargs):
        msg = OffboardControlMode()
        msg.position = kwargs.get("position", False)
        msg.velocity = kwargs.get("velocity", False)
        msg.acceleration = kwargs.get("acceleration", False)
        msg.attitude = kwargs.get("attitude", False)
        msg.body_rate = kwargs.get("body_rate", False)
        msg.thrust_and_torque = kwargs.get("thrust_and_torque", False)
        msg.direct_actuator = kwargs.get("direct_actuator", False)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_trajectory_setpoint(self, **kwargs):
        msg = TrajectorySetpoint()
        # position setpoint is relative to the home position
        msg.position = list( kwargs.get("position_sp", np.nan * np.zeros(3)) + self.home_position )
        msg.velocity = list( kwargs.get("velocity_sp", np.nan * np.zeros(3)) )
        msg.yaw = kwargs.get("yaw_sp", float('nan'))
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {setposition}")
        

def main(args = None):
    rclpy.init(args=args)

    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)

    yolo_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)