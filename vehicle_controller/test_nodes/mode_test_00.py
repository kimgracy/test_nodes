__author__ = "Kyungjun Oh"
__contact__ = "frankok@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
"""msgs for subscription"""
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VtolVehicleStatus
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

# import os, logging, numpy, pymap3d
import os
import logging
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
        self.takeoff_height = 30.0                          # Parameter: MIS_TAKEOFF_ALT

        self.bezier_threshold_speed = 0.1
        self.mc_start_speed = 0.001
        self.mc_end_speed = 0.001
        self.fw_speed = 17.0
        self.slow_yaw_speed = 0.05                          # 0.05 rad = 2.86 deg
        self.fast_yaw_speed = 0.1                           # 0.1 rad = 5.73 deg
        
        self.mc_acceptance_radius = 0.3
        self.fw_acceptance_radius = 20.0
        self.acceptance_heading_angle = 0.01                # 0.01 rad = 0.57 deg

        self.transition_end = 5.0                           # 5.0 seconds       # TODO: Do we really need this??
        
        self.failsafe_min_z = 35.0
        self.failsafe_max_z_vel = 3.5
        self.failsafe_max_xy_error = 30.0
        self.failsafe_max_z_error = 10.0

        """
        2. Logging setup
        """
        self.declare_parameter('logging', True)
        self.logging = self.get_parameter('logging').value
        
        if self.logging:
            log_dir = os.path.join(os.getcwd(), 'src/vehicle_controller/test_nodes/log')
            os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, f'log_{self.get_clock().now().nanoseconds}.txt')
            logging.basicConfig(filename=log_file,
                                level=logging.INFO,
                                format='%(asctime)s - %(message)s')
            self.logger = logging.getLogger(__name__)

        """
        3. Load waypoints (GPS)
        """
        self.declare_parameter('num_WP', 0)
        self.num_WP = self.get_parameter('num_WP').value

        self.WP = [np.array([0.0, 0.0, -self.takeoff_height])]
        self.WP_gps = [np.array([0.0, 0.0, 0.0])]
        self.home_position = np.array([0.0, 0.0, 0.0])

        for i in range(1, self.num_WP + 1):
            self.declare_parameter(f'gps_WP{i}', None)

        for i in range(1, self.num_WP + 1):
            wp_position_gps = self.get_parameter(f'gps_WP{i}').value
            if wp_position_gps is not None:
                self.WP_gps.append(np.array(wp_position_gps))

        """
        4. Phase and subphase
        """
        # phase description
        # -2 : after flight
        # -1 : before flight
        # 0 : takeoff and arm
        # i >= 1 : moving toward WP_i
        self.phase = -1

        # subphase description
        # MC : multicopter
        # FW_line: fixed-wing line
        # FW_pturn: fixed-wing pturn
        # transition_forward : transition from MC to FW
        # transition_backward : transition from FW to MC
        # yaw_align : align yaw to the next waypoint
        self.subphase = "MC"

        """
        self.phase      self.subphase
        -1              MC
        0               MC
        1               MC
        1               yaw_align
        2               transition_forward
        2               FW_line
        3               FW_line
        ....            ....
        num_WP          FW_line
        num_WP + 1      transition_backward     (betwen WP_{num_WP} and home)
        num_WP + 1      MC
        -2              MC
        """

        """
        5. State variables
        """
        # vehicle status
        self.vehicle_status = VehicleStatus()
        self.vtol_vehicle_status = VtolVehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        # vehicle position, velocity, yaw
        self.pos = np.array([0.0, 0.0, 0.0])        # local
        self.pos_gps = np.array([0.0, 0.0, 0.0])    # global
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = float('nan')

        # waypoints
        self.previous_goal = None
        self.current_goal = None
        self.mission_yaw = float('nan')

        # Bezier curve
        self.num_bezier = 0
        self.bezier_counter = 0
        self.bezier_points = None
        self.declare_parameter('vmax', 5)
        self.vmax = self.get_parameter('vmax').value # receive from yaml file
        self.bezier_minimum_time = 3.0

        # counter
        self.transition_count = 0
        self.logging_count = 0

        """
        6. Create Subscribers
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
        self.vtol_vehicle_status_subscriber = self.create_subscription(
            VtolVehicleStatus, '/fmu/out/vtol_vehicle_status', self.vtol_vehicle_status_callback, qos_profile
        )

        """
        7. Create Publishers
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
        8. timer setup
        """
        self.time_period = 0.05     # 20 Hz
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)
        self.failsafe_timer = self.create_timer(self.time_period, self.failsafe_timer_callback)
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)

        self.print("Successfully executed: vehicle_controller")
        self.print("Please switch to offboard mode\n")

    """
    Services
    """
    def print(self, *args, **kwargs):
        print(*args, **kwargs)
        if self.logging:
            self.logger.info(*args, **kwargs)

    def convert_global_to_local_waypoint(self, home_position_gps):
        self.home_position = self.pos # set home position
        for i in range(1, self.num_WP + 1):
            # WP_gps = [lat, lon, rel_alt]
            wp_position = p3d.geodetic2ned(self.WP_gps[i][0], self.WP_gps[i][1], self.WP_gps[i][2] + home_position_gps[2],
                                            home_position_gps[0], home_position_gps[1], home_position_gps[2])
            wp_position = np.array(wp_position)
            self.WP.append(wp_position)
        self.WP.append(np.array([0.0, 0.0, -self.takeoff_height]))

    def bezier_curve(self, xi, xf):
        # total time calculation
        total_time = np.linalg.norm(xf - xi) / self.vmax * 2      # Assume that average velocity = vmax / 2
        if total_time <= self.bezier_minimum_time:
            total_time = self.bezier_minimum_time

        direction = np.array((xf - xi) / np.linalg.norm(xf - xi))
        vf = self.mc_end_speed * direction
        if np.linalg.norm(self.vel) < self.bezier_threshold_speed:
            vi = self.mc_start_speed * direction
        else:
            vi = self.vel
        print(f'vi: {vi}, vf: {vf}')

        point1 = xi
        point2 = xi + vi * total_time / 3 # * total_time
        point3 = xf - vf * total_time / 3 # * total_time
        point4 = xf
        print(f'point1: {point1}, point2: {point2}, point3: {point3}, point4: {point4}\n')

        # Bezier curve
        self.num_bezier = int(total_time / self.time_period)
        bezier = np.linspace(0, 1, self.num_bezier).reshape(-1, 1)
        bezier = point4 * bezier**3 +                             \
                3 * point3 * bezier**2 * (1 - bezier) +           \
                3 * point2 * bezier**1 * (1 - bezier)**2 +        \
                1 * point1 * (1 - bezier)**3
        return bezier
        
    def get_bearing_to_next_waypoint(self, now, next):
        now2d = now[0:2]
        next2d = next[0:2]
        direction = (next2d - now2d) / np.linalg.norm(next2d - now2d) # NED frame
        yaw = np.arctan2(direction[1], direction[0])
        return yaw

    """
    Callback functions for the timers
    """
    def offboard_heartbeat_callback(self):
        """offboard heartbeat signal"""
        if self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
            self.publish_offboard_control_mode(position = True)
        elif self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
            self.publish_offboard_control_mode(position = True, velocity = True)
        else: # transition
            self.publish_offboard_control_mode(position = True, velocity = True)
    
    def failsafe_timer_callback(self):
        """failsafe timer"""
        if self.phase > 1 and self.phase < self.num_WP + 1:
            if np.abs(self.pos[2]) < self.failsafe_min_z:
                self.print("Failsafe Trigger: under minimum altitude!!!")
                self.print(f'Altitude: {self.pos[2]}\n')
                
            if np.abs(self.vel[2]) > self.failsafe_max_z_vel:
                self.print("Failsafe Trigger: max z velocity exceeded!!!")
                if self.vel[2] > 0:
                    self.print("Vehicle is descending too fast!!!")
                else:
                    self.print("Vehicle is ascending too fast!!!")
                self.print(f'Z Velocity: {self.vel[2]}\n')
            
            corridor = self.current_goal - self.previous_goal
            closest_point = np.dot(self.pos - self.previous_goal, corridor) / np.dot(corridor, corridor) * corridor + self.previous_goal
            xy_error = np.linalg.norm(self.pos[:2] - closest_point[:2])
            z_error = np.abs(self.pos[2] - closest_point[2])

            if self.subphase == "FW_line" or self.subphase == "FW_pturn" or self.subphase == "MC":
                if xy_error > self.failsafe_max_xy_error:
                    self.print("Failsafe Trigger: max xy error exceeded!!!")
                    self.print(f'XY Error: {xy_error}\n')
                
                if z_error > self.failsafe_max_z_error:
                    self.print("Failsafe Trigger: max z error exceeded!!!")
                    self.print(f'Z Error: {z_error}\n')

    
    def main_timer_callback(self):
        """Callback function for the timer."""
        if self.phase == -1:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                print("Takeoff requested\n")
                self.phase = 0

        elif self.phase == 0:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.print("Takeoff completed")
                self.print("Offboard control mode requested\n")
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                    param1=1.0, # main mode
                    param2=6.0  # offboard
                )
                self.transition_count = 0
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.transition_count += 1
                if self.transition_count >= self.transition_end / self.time_period:
                    self.print("Position control mode requested\n")
                    self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0, # main mode
                    param2=3.0  # position ctl
                )
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
                self.print("Position control mode completed\n")
                self.phase = 1
                self.transition_count = 0
        
        elif self.phase == 1:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
                self.transition_count += 1
                if self.transition_count >= self.transition_end / self.time_period:
                    self.print("Offboard control mode requested\n")
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                        param1=1.0, # main mode
                        param2=6.0  # offboard
                    )
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.print("Landing requested\n")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                # vehicle command send repeatedly until the vehicle is in landing state
                self.phase = -2

        else:
            self.print("Mission completed")
            self.print("Congratulations!\n")
            self.destroy_node()
            rclpy.shutdown()

        # FOR DEBUGGING
        self.logging_count += 1
        if self.logging_count % 100 == 0:        # TODO: 10Hz logging
            self.print(f'Phase: {self.phase}, Subphase: {self.subphase}')
            self.print(f'Mode: {self.vehicle_status.nav_state}, VTOL state: {self.vtol_vehicle_status.vehicle_vtol_state}')
            self.print(f'Position: {self.pos}')
            self.print(f'Velocity: {self.vel}')
            self.print(f'Previous goal: {self.previous_goal}')
            self.print(f'Current goal: {self.current_goal}\n')
            self.logging_count = 0


    """
    Callback functions for subscribers.
    """        
    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg
    
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.vel = np.array([msg.vx, msg.vy, msg.vz])
        self.yaw = msg.heading
        if self.phase != -1:
            # set position relative to the home position after takeoff
            self.pos = self.pos - self.home_position
    
    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position = msg
        self.pos_gps = np.array([msg.lat, msg.lon, msg.alt])

    def vtol_vehicle_status_callback(self, msg):
        self.vtol_vehicle_status = msg
    
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
        msg.yawspeed = kwargs.get("yaw_speed", float('nan'))
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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