# TODO: FIX!!

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

        self.mc_speed = 5.0
        self.fw_speed = 17.0
        self.slow_yaw_speed = 0.02                          # 0.02 rad = 1.15 deg
        self.fast_yaw_speed = 0.1                           # 0.1 rad = 5.73 deg

        self.mc_acceptance_radius = 0.3
        self.fw_acceptance_radius = 20.0
        self.acceptance_heading_angle = 0.01                # 0.01 rad = 0.57 deg

        self.transition_end = 5.0                           # 5.0 seconds       # TODO: Do we really need this??
        self.back_transition_ratio = 0.400                  # WP6 to WP7 internal divide ratio

        self.pturn_radius = 50.0
        self.num_pturn_points = 5

        self.failsafe_min_z = 8.0
        self.failsafe_max_z_vel = 3.5
        self.failsafe_max_xy_error = 20.0
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

        self.WP = [np.array([0.0, 0.0, -self.takeoff_height])]
        self.WP_gps = [np.array([0.0, 0.0, 0.0])]
        self.home_position = np.array([0.0, 0.0, 0.0])

        for i in range(1, 8):
            self.declare_parameter(f'gps_WP{i}', None)

        for i in range(1, 8):
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
        3               FW_pturn        # WP1 -> WP2 -> pturn -> (WP2') -> WP3
        3               FW_line
        4               FW_pturn        # WP2 -> WP3 -> pturn -> (WP3') -> WP4
        4               FW_line
        5               FW_line
        6               FW_pturn        # WP4 -> WP5 -> pturn -> WP6 -> WP7
        6               FW_line
        7               FW_line
        7               transition_backward
        7               MC
        8               yaw_align
        8               MC
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
        self.previous_goal_pos = None
        self.current_goal_pos = None
        self.current_goal_vel = None
        self.mission_yaw = float('nan')

        # pturn
        self.pturn_pos_trajectory = []
        self.pturn_vel_trajectory = []

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
        for i in range(1, 8):
            # WP_gps = [lat, lon, rel_alt]
            wp_position = p3d.geodetic2ned(self.WP_gps[i][0], self.WP_gps[i][1], self.WP_gps[i][2] + home_position_gps[2],
                                            home_position_gps[0], home_position_gps[1], home_position_gps[2])
            wp_position = np.array(wp_position)
            self.WP.append(wp_position)
        self.WP.append(np.array([0.0, 0.0, -self.takeoff_height]))
        
    def get_bearing_to_next_waypoint(self, now, next):
        now2d = now[0:2]
        next2d = next[0:2]
        direction = (next2d - now2d) / np.linalg.norm(next2d - now2d) # NED frame
        yaw = np.arctan2(direction[1], direction[0])
        return yaw

    def generate_pturn_trajectory(self, pointA, pointB, pointC, pointD):
        # pointA -> pointB -> pturn -> pointC -> pointD
        pointA2d, pointAz = pointA[0:2], pointA[2]
        pointB2d, pointBz = pointB[0:2], pointB[2]
        pointC2d, pointCz = pointC[0:2], pointC[2]
        pointD2d, pointDz = pointD[0:2], pointD[2]

        # pturn direction
        u = (pointB2d - pointA2d) / np.linalg.norm(pointB2d - pointA2d)
        v = (pointC2d - pointD2d) / np.linalg.norm(pointC2d - pointD2d)
        alpha = np.arccos(np.dot(u, v)) / 2     # half angle between u and v
        clockwise = np.linalg.det([u, v]) < 0

        # intersect point
        intersect_point = pointB2d
        if np.linalg.norm(pointB2d - pointC2d) > 1e-1:  # pointB2d != pointC2d
            # pointA2d, pointB2d and pointC2d, pointD2d line intersection
            a1, a2, a3 = pointA2d[1] - pointB2d[1], pointB2d[0] - pointA2d[0], pointA2d[1] * pointB2d[0] - pointA2d[0] * pointB2d[1]
            b1, b2, b3 = pointC2d[1] - pointD2d[1], pointD2d[0] - pointC2d[0], pointC2d[1] * pointD2d[0] - pointC2d[0] * pointD2d[1]
            det = a1 * b2 - a2 * b1
            intersect_point = np.array([(a3 * b2 - a2 * b3) / det, (a1 * b3 - a3 * b1) / det])
        
        # pturn center, contact point
        pturn_center = intersect_point + self.pturn_radius / np.sin(alpha) * (u + v) / np.linalg.norm(u + v)
        contact_point1 = intersect_point + self.pturn_radius / np.tan(alpha) * u
        contact_point2 = intersect_point + self.pturn_radius / np.tan(alpha) * v
        center_to_contact = contact_point1 - pturn_center
        start_angle = np.arctan2(center_to_contact[1], center_to_contact[0])

        # pturn 2d trajectory
        if clockwise:
            theta = np.linspace(start_angle, start_angle - (np.pi + 2 * alpha), self.num_pturn_points - 1)
            pturn_points_2d = pturn_center + self.pturn_radius * np.array([np.cos(theta), np.sin(theta)]).T
            pturn_vel_2d = self.pturn_radius * (np.pi + 2 * alpha) * np.array([np.sin(theta), -np.cos(theta)]).T
        else:
            theta = np.linspace(start_angle, start_angle + (np.pi + 2 * alpha), self.num_pturn_points - 1)
            pturn_points_2d = pturn_center + self.pturn_radius * np.array([np.cos(theta), np.sin(theta)]).T
            pturn_vel_2d = self.pturn_radius * (np.pi + 2 * alpha) * np.array([-np.sin(theta), np.cos(theta)]).T
        pturn_points_2d = np.append(pturn_points_2d, pointC2d.reshape(1, -1), axis=0)
        pturn_vel_2d = np.append(pturn_vel_2d, pturn_vel_2d[-1].reshape(1, -1), axis=0)

        # pturn z trajectory
        horizontal_dist = np.linalg.norm(contact_point1 - pointB2d) + np.linspace(0, self.pturn_radius * (np.pi + 2 * alpha), self.num_pturn_points - 1)
        horizontal_dist = np.append(horizontal_dist, horizontal_dist[-1] + np.linalg.norm(pointC2d - contact_point2))
        if np.abs(pointCz - pointDz) > 1e-1:  # pointCz != pointDz
            c_to_d = np.linalg.norm(pointC2d - pointD2d)
            pointCz = (pointBz * c_to_d + pointDz * horizontal_dist[-1]) / (c_to_d + horizontal_dist[-1])
        # pointBz -> pointCz (proportional to horizontal_dist)
        pturn_points_z = (pointCz - pointBz) / horizontal_dist[-1] * horizontal_dist + pointBz
        # pturn_vel_z = (pointCz - pointBz) / horizontal_dist[-1] * self.pturn_radius * (np.pi + 2 * alpha) * np.ones(self.num_pturn_points)
        pturn_vel_z = (pointCz - pointBz) / horizontal_dist[-1] * self.pturn_radius * (np.pi + 2 * alpha) * np.ones(self.num_pturn_points - 1)
        pturn_vel_z = np.append(pturn_vel_z, 0)     # for safety

        # pturn trajectory
        pturn_points = [np.array([pturn_points_2d[i][0], pturn_points_2d[i][1], pturn_points_z[i]]) for i in range(self.num_pturn_points)]
        pturn_vel = [np.array([pturn_vel_2d[i][0], pturn_vel_2d[i][1], pturn_vel_z[i]]) for i in range(self.num_pturn_points)]
        pturn_vel = [self.fw_speed * vel / np.linalg.norm(vel) for vel in pturn_vel]

        return pturn_points, pturn_vel

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
        if self.phase > 0:
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
            
            corridor = self.current_goal_pos - self.previous_goal_pos
            closest_point = np.dot(self.pos - self.previous_goal_pos, corridor) / np.dot(corridor, corridor) * corridor + self.previous_goal_pos
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
                self.convert_global_to_local_waypoint(self.pos_gps)
                self.previous_goal_pos = self.current_goal_pos
                self.current_goal_pos = self.WP[0]
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
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # vehicle command send repeatedly until the vehicle is in offboard mode")
                self.print("Change to offboard mode completed")
                self.print("WP1 requested\n")
                self.previous_goal_pos = self.current_goal_pos
                self.current_goal_pos = self.WP[1]
                self.phase = 1

        elif self.phase == 1:
            if self.subphase == "MC":
                self.publish_trajectory_setpoint(
                    position_sp = self.pos + (self.current_goal_pos - self.pos) / np.linalg.norm(self.current_goal_pos - self.pos) * self.mc_speed) # slow mc flight
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.mc_acceptance_radius:
                    self.print(f'WP{self.phase} reached')
                    self.print("Yaw alignment requested\n")
                    self.previous_goal_pos = self.current_goal_pos
                    self.current_goal_pos = self.WP[2]
                    self.mission_yaw = self.get_bearing_to_next_waypoint(self.pos, self.current_goal_pos)
                    self.subphase = "yaw_align"

            elif self.subphase == "yaw_align":
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD    \
                    and self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
                    # offboard mode and MC mode
                    self.publish_trajectory_setpoint(
                        position_sp = self.pos,
                        yaw_sp = self.yaw + np.sign(np.sin(self.mission_yaw - self.yaw)) * self.slow_yaw_speed)   # slow smooth yaw alignment
                    if np.abs(self.yaw - self.mission_yaw) < self.acceptance_heading_angle:
                        self.print("Yaw alignment completed")
                        self.print("Position control mode & Transition to FW requested\n")
                        self.publish_vehicle_command(
                            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                            param1=1.0, # main mode
                            param2=3.0  # position ctl
                        )
                        self.publish_vehicle_command(
                            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 
                            param1=float(VtolVehicleStatus.VEHICLE_VTOL_STATE_FW), 
                            param2=0.0  # normal transition
                        )
                elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL    \
                    and self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW:
                    # vehicle command send repeatedly until the vehicle is in position control mode and forward transition mode
                    self.print("Change to position control mode completed")
                    self.print("Transitioning to FW mode...\n")
                    self.phase = 2
                    self.subphase = "transition_forward"
                else:
                    self.print("Transition Forward Error Detected!!")
                    self.print(f'nav_state: {self.vehicle_status.nav_state}, vtol_state: {self.vtol_vehicle_status.vehicle_vtol_state}')
                    self.print("Error Handling...")
                    if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_POSCTL:
                        self.print("Position control mode requested\n")
                        self.publish_vehicle_command(
                            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                            param1=1.0, # main mode
                            param2=3.0  # position ctl
                        )
                    if self.vtol_vehicle_status.vehicle_vtol_state != VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW:
                        self.print("Transition to FW requested\n")
                        self.publish_vehicle_command(
                            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 
                            param1=float(VtolVehicleStatus.VEHICLE_VTOL_STATE_FW), 
                            param2=0.0  # normal transition
                        )

        elif self.phase == 2:
            if self.subphase == "transition_forward":
                if self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
                    if self.transition_count >= self.transition_end / self.time_period:
                        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
                            self.print("Transition to FW completed")
                            self.print("Offboard mode requested\n")
                            self.publish_vehicle_command(
                                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                                param1=1.0, # main mode
                                param2=6.0  # offboard
                            )
                        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                            # vehicle command send repeatedly until the vehicle is in offboard mode
                            self.print("Change to Offboard mode completed")
                            self.print("WP2 requested\n")
                            self.current_goal_vel = self.fw_speed * (self.current_goal_pos - self.previous_goal_pos) / np.linalg.norm(self.current_goal_pos - self.previous_goal_pos)
                            self.subphase = "FW_line"
                        else:
                            self.print("Return to Offboard mode Critical Error Detected!!!\n")
                    self.transition_count += 1
                    
            elif self.subphase == "FW_line":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'WP{self.phase} reached')
                    self.print("Pturn trajectory requested\n")
                    self.pturn_pos_trajectory, self.pturn_vel_trajectory = self.generate_pturn_trajectory(self.WP[1], self.WP[2], self.WP[2], self.WP[3])
                    self.pturn_count = 0
                    self.previous_goal_pos = self.current_goal_pos
                    self.current_goal_pos = self.pturn_pos_trajectory[self.pturn_count]
                    self.current_goal_vel = self.pturn_vel_trajectory[self.pturn_count]
                    self.phase = 3
                    self.subphase = "FW_pturn"
        
        elif self.phase == 3:
            if self.subphase == "FW_pturn":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'Pturn point {self.pturn_count + 1} reached\n')
                    if self.pturn_count < self.num_pturn_points - 1:
                        self.pturn_count += 1
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.pturn_pos_trajectory[self.pturn_count]
                        self.current_goal_vel = self.pturn_vel_trajectory[self.pturn_count]
                    else:
                        self.print("WP3 requested\n")
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.WP[3]
                        self.current_goal_vel = self.fw_speed * (self.current_goal_pos - self.previous_goal_pos) / np.linalg.norm(self.current_goal_pos - self.previous_goal_pos)
                        self.subphase = "FW_line"

            elif self.subphase == "FW_line":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'WP{self.phase} reached')
                    self.print("Pturn trajectory requested\n")
                    self.pturn_pos_trajectory, self.pturn_vel_trajectory = self.generate_pturn_trajectory(self.WP[2], self.WP[3], self.WP[3], self.WP[4])
                    self.pturn_count = 0
                    self.previous_goal_pos = self.current_goal_pos
                    self.current_goal_pos = self.pturn_pos_trajectory[self.pturn_count]
                    self.current_goal_vel = self.pturn_vel_trajectory[self.pturn_count]
                    self.phase = 4
                    self.subphase = "FW_pturn"
        
        elif self.phase == 4:
            if self.subphase == "FW_pturn":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'Pturn point {self.pturn_count + 1} reached\n')
                    if self.pturn_count < self.num_pturn_points - 1:
                        self.pturn_count += 1
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.pturn_pos_trajectory[self.pturn_count]
                        self.current_goal_vel = self.pturn_vel_trajectory[self.pturn_count]
                    else:
                        self.print("WP4 requested\n")
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.WP[4]
                        self.current_goal_vel = self.fw_speed * (self.current_goal_pos - self.previous_goal_pos) / np.linalg.norm(self.current_goal_pos - self.previous_goal_pos)
                        self.subphase = "FW_line"
            
            elif self.subphase == "FW_line":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'WP{self.phase} reached')
                    self.print("WP5 requested\n")
                    self.previous_goal_pos = self.current_goal_pos
                    self.current_goal_pos = self.WP[5]
                    self.current_goal_vel = self.fw_speed * (self.current_goal_pos - self.previous_goal_pos) / np.linalg.norm(self.current_goal_pos - self.previous_goal_pos)
                    self.phase = 5
            
        elif self.phase == 5:
            if self.subphase == "FW_line":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'WP{self.phase} reached\n')
                    self.print("Pturn trajectory requested\n")
                    self.pturn_pos_trajectory, self.pturn_vel_trajectory = self.generate_pturn_trajectory(self.WP[4], self.WP[5], self.WP[6], self.WP[7])
                    self.pturn_count = 0
                    self.previous_goal_pos = self.current_goal_pos
                    self.current_goal_pos = self.pturn_pos_trajectory[self.pturn_count]
                    self.current_goal_vel = self.pturn_vel_trajectory[self.pturn_count]
                    self.phase = 6
                    self.subphase = "FW_pturn"
        
        elif self.phase == 6:
            if self.subphase == "FW_pturn":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    self.print(f'Pturn point {self.pturn_count + 1} reached\n')
                    if self.pturn_count < self.num_pturn_points - 1:
                        self.pturn_count += 1
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.pturn_pos_trajectory[self.pturn_count]
                        self.current_goal_vel = self.pturn_vel_trajectory[self.pturn_count]
                    else:
                        self.print(f'WP{self.phase} reached')
                        self.print("WP6 ~ WP7 requested\n")
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.WP[6] * (1 - self.back_transition_ratio) + self.WP[7] * self.back_transition_ratio     # internal divide point
                        self.current_goal_vel = self.fw_speed * (self.current_goal_pos - self.previous_goal_pos) / np.linalg.norm(self.current_goal_pos - self.previous_goal_pos)
                        self.phase = 7
                        self.subphase = "FW_line"
        
        elif self.phase == 7:
            if self.subphase == "FW_line":
                self.publish_trajectory_setpoint(
                    position_sp = self.current_goal_pos,
                    velocity_sp = self.current_goal_vel
                )
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.fw_acceptance_radius:
                    if self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
                        self.print("Transition to MC requested\n")
                        self.publish_vehicle_command(
                            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 
                            param1=float(VtolVehicleStatus.VEHICLE_VTOL_STATE_MC), 
                            param2=0.0  # normal transition
                        )
                    elif self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_MC:
                        # vehicle command send repeatedly until the vehicle is in back transition state
                        self.print("Transitioning to MC mode...\n")
                        self.subphase = "transition_backward"
                    else:
                        self.print("Transition Backward Critical Error Detected!!!\n")

            elif self.subphase == "transition_backward":
                if self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
                    self.print("Transition to MC completed")
                    self.print("WP7 requested\n")
                    self.previous_goal_pos = self.current_goal_pos
                    self.current_goal_pos = self.WP[7]
                    self.current_goal_vel = None
                    self.subphase = "MC"
            
            elif self.subphase == "MC":
                if self.subphase == "MC":
                    self.publish_trajectory_setpoint(
                        position_sp = self.pos + (self.current_goal_pos - self.pos) / np.linalg.norm(self.current_goal_pos - self.pos) * self.mc_speed) # slow mc flight
                    if np.linalg.norm(self.pos - self.current_goal_pos) < self.mc_acceptance_radius:
                        self.print(f'WP{self.phase} reached')
                        self.print("Yaw alignment requested\n")
                        self.previous_goal_pos = self.current_goal_pos
                        self.current_goal_pos = self.WP[8]
                        self.mission_yaw = self.get_bearing_to_next_waypoint(self.pos, self.current_goal_pos)
                        self.phase = 8
                        self.subphase = "yaw_align"
        
        elif self.phase == 8:
            if self.subphase == "yaw_align":
                self.publish_trajectory_setpoint(
                    position_sp = self.pos,
                    yaw_sp = self.yaw + np.sign(np.sin(self.mission_yaw - self.yaw)) * self.fast_yaw_speed)   # fast smooth yaw alignment
                if np.abs(self.yaw - self.mission_yaw) < self.acceptance_heading_angle:
                    self.print("Yaw alignment completed")
                    self.print("Home position requested\n")
                    self.subphase = "MC"
                
            elif self.subphase == "MC":
                self.publish_trajectory_setpoint(
                        position_sp = self.pos + (self.current_goal_pos - self.pos) / np.linalg.norm(self.current_goal_pos - self.pos) * self.mc_speed) # slow mc flight
                if np.linalg.norm(self.pos - self.current_goal_pos) < self.mc_acceptance_radius:
                    if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
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
            self.print(f'Previous goal: {self.previous_goal_pos}')
            self.print(f'Current goal: {self.current_goal_pos}')
            self.print(f'Current goal velocity: {self.current_goal_vel}\n')
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