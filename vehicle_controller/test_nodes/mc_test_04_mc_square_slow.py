__author__ = "Juyong Shin"
__contact__ = "juyong3393@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

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
        2. Set waypoints
        """
        self.WP = [np.array([0.0, 0.0, 0.0])]
        self.declare_parameters(
            namespace='',
            parameters=[
                ('WP1', None),
                ('WP2', None),
                ('WP3', None),
                ('WP4', None),
            ])

        for i in range(1, 5):
            wp_position = self.get_parameter(f'WP{i}').value
            self.WP.append(np.array(wp_position))

        """
        3. State variables
        """
        # phase description
        # -2 : after flight
        # -1 : before flight
        # 0 : takeoff and arm
        # i >= 1 : moving toward WP_i
        self.phase = -1
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.home_position = np.array([0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0])
        
        self.previous_goal = None
        self.current_goal = None

        # add by chaewon. used for step by step function
        self.start_point = None
        self.setpoint_list = []
        self.step_velocity = 0.0
        self.step_count = 0

        self.time_checker = 0

        """
        4. Create Subscribers
        """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
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
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )

        """
        6. timer setup
        """
        self.offboard_heartbeat = self.create_timer(0.1, self.offboard_heartbeat_callback)
        self.takeoff_timer = self.create_timer(0.5, self.takeoff_and_arm_callback)
        self.main_timer = self.create_timer(0.05, self.main_timer_callback)
        
        print("Successfully executed: vehicle_controller")
        print("Please switch to offboard mode.")
        
    
    """
    Services
    """   
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.phase = -2
    
    # 천천히 비행
    def make_setpoint_list(self, start, finish, v):
        start = np.array(start)
        finish = np.array(finish)
        n = int(np.linalg.norm(finish - start) // (v*0.05))
        # N등분점 생성 + 점 하나 추가
        points = np.linspace(start, finish, num=n+1, endpoint=True)[1:]
        last_point = points[-1] + (points[-1] - points[-2])
        points = np.append(points, [last_point], axis=0)
        # 속도 벡터 생성
        velocity = list(v * (finish - start) / np.linalg.norm(finish - start))
        print([list(point) for point in points])
        print(n)
        print(velocity)
        return [list(point) for point in points], velocity
    
    def step_by_step(self, setpoints, velocity):
        if math.floor(self.step_count) == len(setpoints)-1:
            previous_goal = np.array(setpoints[-2])
            self.publish_trajectory_setpoint(position_sp=previous_goal, velocity_sp=np.array([0.0, 0.0, 0.0]))
            #print(f'finish at point {self.step_count}')
        elif math.floor(self.step_count) >= len(setpoints)-int(1/0.05):
            self.step_count += 0.5
            a = float(1-(self.step_count-(len(setpoints)-int(1/0.05)))*0.05/1)
            velocity = np.array(velocity) * a
            self.publish_trajectory_setpoint(velocity_sp=velocity)
            #print(f'stop at point {self.step_count}')
        else:
            self.step_count += 1
            previous_goal = np.array(setpoints[int(self.step_count)])
            self.publish_trajectory_setpoint(position_sp=previous_goal, velocity_sp=np.array(velocity))
            #print(f'going to point {self.step_count}')

    """
    Callback functions for the timers
    """
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        self.publish_offboard_control_mode(position=True, velocity=True)

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
            self.start_point = self.pos
            self.current_goal = np.array([2.0, 0.0, -5.0])
            self.setpoint_list, self.step_velocity = self.make_setpoint_list(list(self.start_point), list(self.current_goal), 0.5)
            self.phase = 1
        elif self.phase == 1:
            self.step_by_step(self.setpoint_list, self.step_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.start_point = self.pos
                self.current_goal = np.array([2.0, 2.0, -5.0])
                self.setpoint_list, self.step_velocity = self.make_setpoint_list(list(self.start_point), list(self.current_goal), 0.5)
                self.phase = 2
        elif self.phase == 2:
            self.step_by_step(self.setpoint_list, self.step_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.start_point = self.pos
                self.current_goal = np.array([0.0, 2.0, -5.0])
                self.setpoint_list, self.step_velocity = self.make_setpoint_list(list(self.start_point), list(self.current_goal), 0.5)
                self.phase = 3
        elif self.phase == 3:
            self.step_by_step(self.setpoint_list, self.step_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.start_point = self.pos
                self.current_goal = np.array([0.0, 0.0, -5.0])
                self.setpoint_list, self.step_velocity = self.make_setpoint_list(list(self.start_point), list(self.current_goal), 0.5)
                self.phase = 4
        elif self.phase == 4:
            self.step_by_step(self.setpoint_list, self.step_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.land()
                self.phase = -2

        '''
        elif self.phase >= 1:
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                if self.phase == 4:
                    self.land()
                    self.phase = -2
                elif self.phase - math.floor(self.phase) != 0:
                    self.start_point = self.pos
                    self.current_goal = self.WP[int(math.floor(self.phase)) + 1]
                    self.setpoint_list, self.step_velocity = self.make_setpoint_list(list(self.start_point), list(self.current_goal), 1)
                    self.phase += 0.5
                else:
                    self.previous_goal = self.current_goal
                    self.current_goal = self.WP[self.phase + 1]
                    self.publish_trajectory_setpoint(position_sp=self.current_goal)
                    self.phase += 0.5
            else:
                self.step_by_step(self.setpoint_list, self.step_velocity)
        '''
        print(self.pos)
        print(self.phase)

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

    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
