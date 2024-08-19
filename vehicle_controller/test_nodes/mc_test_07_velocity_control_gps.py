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
from px4_msgs.msg import VehicleGlobalPosition
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

# import math, numpy
import math
import numpy as np
import pymap3d as p3d



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
        self.main_timer_period = 0.05

        """
        2. Set waypoints
        """
        self.WP_gps = [np.array([0.0, 0.0, 0.0])]
        self.WP = [np.array([0.0, 0.0, 0.0])]
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gps_WP1', None),
                ('gps_WP2', None),
                ('gps_WP3', None),
                ('velocity', None),
                ('angle', None),
            ])

        for i in range(1, 4):
            wp_position_gps = self.get_parameter(f'gps_WP{i}').value
            self.WP_gps.append(np.array(wp_position_gps))

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
        self.home_position_gps = np.array([0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0])
        self.pos_gps = np.array([0.0, 0.0, 0.0])
        
        self.previous_goal = None
        self.current_goal = None

        # add by chaewon. used for step by step function
        self.start_point = None
        self.set_velocity = self.get_parameter(f'velocity').value # receive from yaml file
        self.max_acceleration = 9.8*np.tan((self.get_parameter(f'angle').value)*np.pi/180)  # 8 degree tilt angle
        self.time_variable = 0

        # stabilize the vehicle before moving
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
        self.Vehicle_Global_Position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile
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
        self.main_timer = self.create_timer(self.main_timer_period, self.main_timer_callback)
        
        print("Successfully executed: vehicle_controller")
        print("Please switch to offboard mode.")
        
    
    """
    Services
    """   
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.phase = -2
    
    # 천천히 비행. velocity control
    '''
    parameters used in the function
    ------------------------------
    self.start_point = None         => start
    self.set_velocity = self.get_parameter(f'velocity').value # receive from yaml file
    self.max_acceleration = 9.8*np.tan((self.get_parameter(f'angle').value)*np.pi/180) # 8 degree tilt angle
    self.time_variable = 0         => time_checker for this function. must be initialized to 0 when the function is called
    '''
    
    def velocity_control(self, start, goal, velocity_constant):
        # start = self.start_point = WP[x-1], goal = WP[x], velocity_constant = self.set_velocity
        t = velocity_constant / self.max_acceleration
        braking_distance = velocity_constant**2/ self.max_acceleration
        distance = np.linalg.norm(self.pos - goal)
        total_distance = np.linalg.norm(start - goal)
        direction = (goal - self.pos) / distance #(np.array(goal) - np.array(self.pos)) / distance

        if 2*braking_distance > total_distance:
            print('braking distance is too long. change the velocity')
            velocity_constant = np.sqrt((total_distance/2) * self.max_acceleration)
            t = velocity_constant / self.max_acceleration
            braking_distance = velocity_constant**2/ self.max_acceleration

        if (distance < 2 * self.mc_acceptance_radius) or (self.time_variable >= 4*t): # stop when the vehicle is close to the goal
            self.publish_trajectory_setpoint(position_sp=goal, velocity_sp=[0.0, 0.0, 0.0])
            print('finish')
        elif np.linalg.norm(self.time_variable - 2*t) < 0.025: # keep going at constant speed
            velocity = direction * float(velocity_constant)
            self.publish_trajectory_setpoint(velocity_sp=velocity)
            if distance < braking_distance + 1*velocity_constant:
                self.time_variable += self.main_timer_period
            print('go')
        elif self.time_variable < self.main_timer_period * 5: # give some time to go forward
            self.time_variable += self.main_timer_period
            a = np.abs(self.velocity_function(self.main_timer_period*5, t, velocity_constant))
            velocity = direction * a
            self.publish_trajectory_setpoint(velocity_sp=velocity)
            print('start')
        else: # accelerating and decelerating
            self.time_variable += self.main_timer_period
            a = np.abs(self.velocity_function(self.time_variable, t, velocity_constant))
            velocity = direction * a
            self.publish_trajectory_setpoint(velocity_sp=velocity)
            print('accelerate or decelerate')
    
    def velocity_function(self, time, t, v):
        x = -np.abs(time - 2*t) + t
        y = (np.abs(x)/x) * (0.5*v - 0.5*(self.max_acceleration/t)*(np.abs(x)-t)**2) + 0.5*v
        z = (-np.abs((time-4*t)*(time))/((time-4*t)*(time))+1) * y
        print(z)
        return float(z)
    
    '''
    def velocity_control(self, start, goal, velocity_constant): 
        # start = self.start_point = WP[x-1], goal = WP[x], velocity_constant = self.set_velocity
        t = velocity_constant / self.max_acceleration
        braking_distance = velocity_constant**2/ self.max_acceleration
        distance = np.linalg.norm(self.pos - goal)
        total_distance = np.linalg.norm(start - goal)
        direction = (goal - self.pos) / distance #(np.array(goal) - np.array(self.pos)) / distance

        if (distance < 1 * self.mc_acceptance_radius): # stop when the vehicle is close to the goal
            self.publish_trajectory_setpoint(position_sp=goal, velocity_sp=[0.0, 0.0, 0.0])
            print('finish')
        elif np.linalg.norm(self.pos - start) < self.mc_acceptance_radius: # give some time to go forward
            self.time_variable += self.main_timer_period
            a = np.abs(self.velocity_function(self.main_timer_period*5, t, velocity_constant))
            velocity = direction * a
            self.publish_trajectory_setpoint(velocity_sp=velocity)
            print('start')
        elif distance < 0.5*total_distance: # after half of the distance
            self.velocity_decrease(self.time_variable, t, velocity_constant, direction, distance, braking_distance)
            print('decelerate')
        else: # before half of the distance
            self.velocity_increase(self.time_variable, t, velocity_constant, direction)
            print('accelerate')

    def velocity_increase(self, time, t, v, direction):
        if self.time_variable <= 2*t:
            self.time_variable += self.main_timer_period
        a = np.abs(self.velocity_function(time, t, v))
        velocity = direction * a
        self.publish_trajectory_setpoint(velocity_sp=velocity)
        return 0
    
    def velocity_decrease(self, time, t, v, direction, distance, braking_distance):
        if distance < braking_distance + self.mc_acceptance_radius:
            self.time_variable -= self.main_timer_period
        a = np.abs(self.velocity_function(time, t, v))
        velocity = direction * a
        self.publish_trajectory_setpoint(velocity_sp=velocity)
        return 0
    
    def velocity_function(self, time, t, v):
        x = -np.abs(time - 2*t) + t
        y = (np.abs(x)/x) * (0.5*v - 0.5*(self.max_acceleration/t)*(np.abs(x)-t)**2) + 0.5*v
        print(y)
        return float(y)
    '''
    
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
            self.home_position_gps = self.pos_gps # set home position gps
            for i in range(1, 4): #set position relative to the start position before takeoff
                wp_position = p3d.geodetic2ned(self.WP_gps[i][0], self.WP_gps[i][1], self.WP_gps[i][2], self.home_position_gps[0], self.home_position_gps[1], self.home_position_gps[2])
                wp_position = np.array(wp_position)
                wp_position[2] = -5.0
                self.WP.append(wp_position)
            print(self.WP)
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
            self.current_goal = self.WP[1]
            self.time_variable = 0
            self.phase = 1
        elif self.phase == 1:
            self.velocity_control(self.start_point, self.current_goal, self.set_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.time_checker += 1
                if self.time_checker > 20:
                    self.time_variable = 0
                    self.start_point = self.pos
                    self.current_goal = self.WP[2]
                    self.phase = 2
                    self.time_checker = 0
        elif self.phase == 2:
            self.velocity_control(self.start_point, self.current_goal, self.set_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.time_checker += 1
                if self.time_checker > 20:
                    self.time_variable = 0
                    self.start_point = self.pos
                    self.current_goal = self.WP[3]
                    self.phase = 3
                    self.time_checker = 0
        elif self.phase == 3:
            self.velocity_control(self.start_point, self.current_goal, self.set_velocity)
            distance = np.linalg.norm(self.pos - self.current_goal)
            if distance < self.mc_acceptance_radius:
                self.time_variable = 0
                self.land()
                self.phase = -2
        
        #print(self.pos)
        print(self.phase)
        #print(self.WP)


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

    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position = msg
        self.pos_gps = np.array([msg.lat, msg.lon, msg.alt])

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
