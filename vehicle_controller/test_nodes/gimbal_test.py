# Just for debugging purposes

__author__ = "Juyong Shin"
__contact__ = "juyong3393@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import GimbalManagerSetManualControl

# import math, numpy
import math
import numpy as np

class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        """
        0. Configure QoS profile for publishing and subscribing
        """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        """
        1. Constants
        """
        self.mc_acceptance_radius = 0.3
        self.acceptance_heading_angle = np.radians(0.5)

        """
        2. State variables
        """
        # phase description
        # -1 : before flight
        # 0 : takeoff and arm
        self.phase = -1
        self.vehicle_status = VehicleStatus()
        
        self.gimbal_control = False
        self.gimbal_pitchangle = 0.0
        self.gimbal_yawangle = 0.0

        """
        4. Create Subscribers
        """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )

        """
        5. Create Publishers
        """
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.gimbal_publisher = self.create_publisher(
            GimbalManagerSetManualControl, '/fmu/in/gimbal_manager_set_manual_control', qos_profile
        )

        """
        6. timer setup
        """
        self.offboard_heartbeat = self.create_timer(0.1, self.offboard_heartbeat_callback)
        self.arm_timer = self.create_timer(0.5, self.arm_callback)
        self.gimbal_timer = self.create_timer(0.5, self.gimbal_timer_callback)
        self.main_timer = self.create_timer(0.5, self.main_timer_callback)

        print("Successfully executed: vehicle_controller")
        print("Please switch to offboard mode.")

    """
    Callback functions for the timers
    """
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True)
        
    def arm_callback(self):
        if self.phase == -1 and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print("Arm")
            # takeoff and arm only if the vehicle is in offboard mode by RC switch
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.phase = 0

    def gimbal_timer_callback(self):
        if self.gimbal_control:
            msg = GimbalManagerSetManualControl()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.origin_sysid = 0
            msg.origin_compid = 0
            msg.flags = GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_ROLL_LOCK \
                        + GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_PITCH_LOCK \
                        + GimbalManagerSetManualControl.GIMBAL_MANAGER_FLAGS_YAW_LOCK
            msg.pitch = float(self.gimbal_pitchangle)
            msg.yaw = float(self.gimbal_yawangle)
            msg.pitch_rate = float('nan')
            msg.yaw_rate = float('nan')
            self.gimbal_publisher.publish(msg)

    def main_timer_callback(self):
        if self.phase == 0:
            self.gimbal_control = True
            self.gimbal_pitchangle = -math.pi/6
            self.gimbal_yawangle = math.pi/2
        print(self.phase)

    """
    Callback functions for subscribers.
    """        
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg

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
    
def main(args = None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)