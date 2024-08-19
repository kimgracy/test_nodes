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
"""msgs for publishing"""
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

# import os, logging, numpy, pymap3d
import os
import logging
import numpy as np
import pymap3d as p3d
from datetime import datetime

# import message for YOLOv5
from my_bboxes_msg.msg import VehiclePhase
from my_bboxes_msg.msg import YoloObstacle

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
        self.takeoff_height = 5.0                          # Parameter: MIS_TAKEOFF_ALT

        self.mc_acceptance_radius = 0.3
        self.acceptance_heading_angle = 0.01                # 0.01 rad = 0.57 deg

        self.bezier_threshold_speed = 0.7
        self.mc_start_speed = 0.001
        self.mc_end_speed = 0.001

        self.fast_yaw_speed = 0.2                           # 0.1 rad = 5.73 deg


        self.declare_parameter('logging', True)
        self.logging = self.get_parameter('logging').value
        if self.logging:
            log_dir = os.path.join(os.getcwd(), 'src/vehicle_controller/test_nodes/log')
            os.makedirs(log_dir, exist_ok=True)
            current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            log_file = os.path.join(log_dir,  f'log_{current_time}.txt')
            logging.basicConfig(filename=log_file,
                                level=logging.INFO,
                                format='%(asctime)s - %(message)s')
            self.logger = logging.getLogger(__name__)


        """
        2. Load waypoints (GPS)
        """
        self.WP = [np.array([0.0, 0.0, -self.takeoff_height])]
        self.WP_gps = [np.array([0.0, 0.0, 0.0])]
        self.home_position = np.array([0.0, 0.0, 0.0])

        self.declare_parameters(
            namespace='',
            parameters=[
                ('vmax', None),
                ('gps_WP1', None),
                ('gps_WP2', None)
            ])

        for i in range(1, 3):
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
        self.subphase = ''

        # vehicle status
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        # vehicle position
        self.pos = np.array([0.0, 0.0, 0.0])
        self.pos_gps = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0.0
        
        # waypoints
        self.previous_goal = None
        self.current_goal = None
        self.mission_yaw = float('nan')

        # Bezier curve
        self.num_bezier = 0
        self.bezier_counter = 0
        self.bezier_points = None
        self.vmax = self.get_parameter('vmax').value # receive from yaml file
        self.bezier_minimum_time = 3.0

        # YOLOv5
        self.obstacle_label = ''
        self.obstacle_x = 0
        self.obstacle_y = 0
        self.obstacle_orientation = ''
        self.WP_yolo = [np.array([0.0, 0.0, 0.0]),np.array([0.0, 0.0, 0.0]),np.array([0.0, 0.0, 0.0]),np.array([0.0, 0.0, 0.0])]
        self.left_or_right = 0
        self.first_ladder_detected = False
        self.ladder_detected = 0
        self.yolo_wp_chacker = 0

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
        self.yolo_obstacle_subscriber = self.create_subscription(  # YOLOv5
            YoloObstacle, '/yolo_obstacle', self.yolo_obstacle_callback, qos_profile
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
        self.vehicle_phase_publisher = self.create_publisher(  # YOLOv5
            VehiclePhase, '/vehicle_phase', qos_profile
        )

        """
        6. timer setup
        """
        self.time_period = 0.05     # 20 Hz
        self.offboard_heartbeat = self.create_timer(self.time_period, self.offboard_heartbeat_callback)
        self.main_timer = self.create_timer(self.time_period, self.main_timer_callback)
        self.vehicle_phase_publisher_timer = self.create_timer(self.time_period, self.vehicle_phase_publisher_callback)  # YOLOv5
        
        print("Successfully executed: vehicle_controller")
        print("Please switch to offboard mode.\n")
        
    
    """
    Services
    """   
    def print(self, *args, **kwargs):
        print(*args, **kwargs)
        if self.logging:
            self.logger.info(*args, **kwargs)
    
    def convert_global_to_local_waypoint(self, home_position_gps):
        self.bezier_counter = 0
        self.home_position = self.pos # set home position
        for i in range(1, 3):
            # WP_gps = [lat, lon, rel_alt]
            wp_position = p3d.geodetic2ned(self.WP_gps[i][0], self.WP_gps[i][1], self.WP_gps[i][2] + home_position_gps[2],
                                            home_position_gps[0], home_position_gps[1], home_position_gps[2])
            wp_position = np.array(wp_position)
            self.WP.append(wp_position)
        self.WP.append(np.array([0.0, 0.0, -self.takeoff_height]))

    def bezier_curve(self, xi, xf, vmax):
        # reset counter
        self.bezier_counter = 0
        # total time calculation
        total_time = np.linalg.norm(xf - xi) / vmax * 2      # Assume that average velocity = vmax / 2.     real velocity is lower then vmax
        if total_time <= self.bezier_minimum_time:
            total_time = self.bezier_minimum_time

        direction = np.array((xf - xi) / np.linalg.norm(xf - xi))
        vf = self.mc_end_speed * direction
        if np.linalg.norm(self.vel) < self.bezier_threshold_speed:
            vi = self.mc_start_speed * direction
        else:
            vi = self.vel
            self.bezier_counter = int(1/self.time_period) - 1
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
        self.publish_offboard_control_mode(position=True)

    # YOLOv5
    def vehicle_phase_publisher_callback(self):
        msg = VehiclePhase()
        msg.phase = str(self.phase)
        msg.subphase = str(self.subphase)
        self.vehicle_phase_publisher.publish(msg)

    def camera_callback(self):
        pass

    def main_timer_callback(self):
        if self.phase == -1:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.print('\n<< yolo_test_07_bezier >>\n\n')
                self.print("Takeoff requested\n")
                self.convert_global_to_local_waypoint(self.pos_gps)
                self.previous_goal = self.current_goal
                self.current_goal = self.WP[0]
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
                self.previous_goal = self.current_goal
                self.current_goal = self.WP[1]
                self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, self.vmax)
                self.print('\n[phase go to 7]\n')
                self.phase = 7

        elif self.phase == 7:
            if self.bezier_counter == self.num_bezier - int(1 / self.time_period) - 1:
                self.publish_trajectory_setpoint(position_sp=self.current_goal)
            else:
                self.publish_trajectory_setpoint(position_sp=self.bezier_points[self.bezier_counter])
                self.bezier_counter += 1
                
            if np.linalg.norm(self.pos - self.current_goal) < self.mc_acceptance_radius:
                self.print("WP1 reached")
                self.print("WP2 requested\n")
                self.previous_goal = self.current_goal
                self.current_goal = self.WP[2]
                self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, 2)
                self.phase = 8
                self.print('\n[phase go to 8]\n')
                self.subphase = 'go_slow'


        elif self.phase == 8:
            if self.subphase == 'go_slow':
                # go to wp8
                if self.bezier_counter == self.num_bezier - int(1 / self.time_period) - 1:
                    self.publish_trajectory_setpoint(position_sp=self.current_goal)
                else:
                    self.publish_trajectory_setpoint(position_sp=self.bezier_points[self.bezier_counter])
                    self.bezier_counter += 1
                
                # detect obstacle. 'pause'
                if self.obstacle_label != '':
                    self.ladder_detected += 1
                    self.print(f'Detected obstacle: {self.obstacle_label}. {self.ladder_detected} times')
                    self.obstacle_label = ''
                    if self.ladder_detected >= 10:
                        self.ladder_detected = 0
                        self.previous_goal = self.pos
                        self.current_goal = self.pos + np.array([(1.0)*np.cos(self.yaw+(np.pi/2)), (1.0)*np.sin(self.yaw+(np.pi/2)), 0.0])
                        self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, 2)
                        self.print('\n[subphase to pause]\n')
                        self.subphase = 'pause'

                # if reach the wp8 without confronting obstacle
                if np.linalg.norm(self.pos - self.current_goal) < self.mc_acceptance_radius:
                    self.print("WP2 reached")
                    self.print("Home position requested\n")
                    self.previous_goal = self.current_goal
                    # landing
                    self.print('\n[phase go to 9]\n')
                    self.phase = 9
            
            elif self.subphase == 'pause':
                # pause. calculate the angle and distance to align with the vactor of wp7 to wp8  =>  'align'
                # stop slowly
                if self.bezier_counter == self.num_bezier - int(1 / self.time_period) - 1:
                    self.publish_trajectory_setpoint(position_sp=self.current_goal)
                else:
                    self.publish_trajectory_setpoint(position_sp=self.bezier_points[self.bezier_counter])
                    self.bezier_counter += 1
                
                if np.linalg.norm(self.pos - self.current_goal) < self.mc_acceptance_radius:
                    # calculate the angle and distance to align with the vactor of wp7 to wp8
                    self.mission_yaw = self.get_bearing_to_next_waypoint(self.pos, self.current_goal)
                    self.previous_goal = self.current_goal
                    # 점과직선사이거리
                    direction = self.WP[2] - self.WP[1]
                    t = np.dot(self.pos - self.WP[1], direction) / np.dot(direction, direction)
                    self.current_goal = self.WP[1] + t * direction
                    self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, 2)
                    self.print('\n[subphase to align]\n')
                    self.subphase = 'align'
            
            elif self.subphase == 'align':
                # align along the path from 7 to 8  =>  'detecting_obstacle'
                print('|', end='')
                if self.bezier_counter == self.num_bezier - int(1 / self.time_period) - 1:
                    self.publish_trajectory_setpoint(position_sp=self.current_goal, yaw_sp = self.yaw + np.sign(np.sin(self.mission_yaw - self.yaw)) * self.fast_yaw_speed)
                else:
                    self.publish_trajectory_setpoint(
                        position_sp = self.bezier_points[self.bezier_counter],
                        yaw_sp = self.yaw + np.sign(np.sin(self.mission_yaw - self.yaw)) * self.fast_yaw_speed)   # slow smooth yaw alignment
                    self.bezier_counter += 1
                
                if (np.abs(self.yaw - self.mission_yaw) < self.acceptance_heading_angle) and np.linalg.norm(self.pos - self.current_goal) < self.mc_acceptance_radius:
                    self.print("Alignment completed. Detecting obstacle requested\n")
                    self.print('\n[subphase to detecting_obstacle]\n')
                    self.subphase = 'detecting_obstacle'
                    self.ladder_detect_attempts = 0
                    self.ladder_detected = 0
                    self.first_ladder_detected = False
            
            elif self.subphase == 'detecting_obstacle':
                # detecting obstacle for 5 seconds  =>  create the avoidance path  =>  'avoiding_obstacle'
                self.publish_trajectory_setpoint(position_sp=self.current_goal, yaw_sp=self.mission_yaw)

                # Check for the first detection of ladder-truck
                if not self.first_ladder_detected and self.obstacle_label != '':
                    self.first_ladder_detected = True
                    self.obstacle_label = ''
                    self.print('First detection of ladder-truck.')

                # Start counting attempts only after the first detection
                if self.first_ladder_detected:
                    if self.obstacle_label != '':
                        self.print(f'Detected obstacle: {self.obstacle_label}')
                        self.obstacle_label = ''
                        self.ladder_detected += 1
                        # chack the orientation of the obstacle
                        if self.obstacle_orientation == 'left':
                            self.left_or_right -= 1
                        else: # right
                            self.left_or_right += 1
                        # create the avoidance path
                        if self.ladder_detected >= 50:
                            self.print(f'Ladder-truck Orientation: {self.left_or_right}')
                            if self.left_or_right < 0: # obstacle is on the left side
                                self.WP_yolo[0] = self.pos
                                self.WP_yolo[1] = self.pos + np.array([(2.0)*np.cos(self.yaw+(np.pi/2)), (2.0)*np.sin(self.yaw+(np.pi/2)), 0.0])
                                self.WP_yolo[2] = self.WP[2] + np.array([(2.0)*np.cos(self.yaw+(np.pi/2)), (2.0)*np.sin(self.yaw+(np.pi/2)), 0.0])
                                self.WP_yolo[3] = self.WP[2]
                            else:  # obstacle is on the right side
                                self.WP_yolo[0] = self.pos
                                self.WP_yolo[1] = self.pos + np.array([(2.0)*np.cos(self.yaw-(np.pi/2)), (2.0)*np.sin(self.yaw-(np.pi/2)), 0.0])
                                self.WP_yolo[2] = self.WP[2] + np.array([(2.0)*np.cos(self.yaw-(np.pi/2)), (2.0)*np.sin(self.yaw-(np.pi/2)), 0.0])
                                self.WP_yolo[3] = self.WP[2]
                            self.ladder_detected = 0
                            self.left_or_right = 0
                            self.previous_goal = self.current_goal
                            self.current_goal = self.WP_yolo[1]
                            self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, self.vmax)
                            self.print('\n[subphase to avoiding_obstacle]\n')
                            self.subphase = 'avoiding_obstacle'

            elif self.subphase == 'avoiding_obstacle':
                # go along with ractangle path. but if you find obstacle being in front of you  =>  'pause'.
                if np.linalg.norm(self.pos - self.current_goal) < self.mc_acceptance_radius:
                    if self.yolo_wp_chacker == 3:
                        self.print("WP2 reached")
                        self.print("Home position requested\n")
                        self.previous_goal = self.current_goal
                        # landing
                        self.print('\n[phase to 9]\n')
                        self.phase = 9
                    else:
                        self.print(f"yolo wp{self.yolo_wp_chacker} reached")
                        self.yolo_wp_chacker += 1
                        self.previous_goal = self.current_goal
                        self.current_goal = self.WP_yolo[self.yolo_wp_chacker]
                        self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, self.vmax)
                else:
                    if self.bezier_counter == self.num_bezier - int(1 / self.time_period) - 1:
                        self.publish_trajectory_setpoint(position_sp=self.current_goal)
                    else:
                        self.publish_trajectory_setpoint(position_sp=self.bezier_points[self.bezier_counter])
                        self.bezier_counter += 1
                    
                    if (self.obstacle_label != '') and (self.obstacle_x<330 and self.obstacle_x>310):
                        self.ladder_detected += 1
                        self.print(f'Detected obstacle again: {self.obstacle_label}. {self.ladder_detected} times')
                        self.obstacle_label = ''
                        if self.ladder_detected >= 10:
                            self.ladder_detected = 0
                            self.previous_goal = self.pos
                            self.current_goal = self.pos + np.array([(1.0)*np.cos(self.yaw+(np.pi/2)), (1.0)*np.sin(self.yaw+(np.pi/2)), 0.0])
                            self.bezier_points = self.bezier_curve(self.previous_goal, self.current_goal, 2)
                            self.print('\n[subphase to pause]\n')
                            self.subphase = 'pause'

        elif self.phase == 9:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.print("Reached the goal")
            self.print("Landing requested\n")
            self.phase = -2
        
        else:
            self.print("Mission completed")
            self.print("Congratulations!\n")
            self.destroy_node()
            rclpy.shutdown()

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
    
    # YOLOv5. size of picture is 640*480. x 320 기준으로 판단 
    def yolo_obstacle_callback(self, msg):
        self.obstacle_label = msg.label
        self.obstacle_x = int(msg.x)
        self.obstacle_y = int(msg.y)
        if self.obstacle_x < 320:
            self.obstacle_orientation = 'left'
        else:
            self.obstacle_orientation = 'right'

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
