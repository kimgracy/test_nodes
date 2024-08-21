import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_msgs.msg import TFMessage as TfMsg
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float32MultiArray

def quat2R(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                           
    return rot_matrix

class TagPublisher(Node):
    def __init__(self):
        super().__init__('tag_pose')        
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        
        self.drone_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.att_callback,
            qos_profile
        )

        self.drone_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.gps_callback,
            qos_profile
        )
        
        self.tag_sub = self.create_subscription(
            TfMsg,
            'tf',
            self.tag_callback,
            10
        )
        
        self.tag_world_pub = self.create_publisher(Float32MultiArray, 'bezier_waypoint/raw', 10)
        self.last_tag = np.array([0,0,0])
        self.detect = False
        
        
    def tag_callback(self, msg):
        try:
            if self.detect == False:
                self.detect = True
                self.get_logger().info("Tag detected")
            transform = msg.transforms[0].transform
            tag_pose = transform.translation
            tag_q = transform.rotation
            
            rotation = quat2R(self.drone_q)

            tag_body = np.array([-tag_pose.y, tag_pose.x, tag_pose.z])   #changed because 90degree rotation.
            drone2tag_world = np.matmul(rotation,tag_body)
            tag_world = drone2tag_world+self.drone_world
            self.last_tag = tag_world
           
            tag_world_msg = Float32MultiArray()
            tag_world_msg.data = [tag_world[0], tag_world[1], tag_world[2]+0.4, 0., 0., 0.5] # in order of xf and vf
            
            self.tag_world_pub.publish(tag_world_msg)

        except:
            if self.last_tag[0]:
                tag_world = self.last_tag
                tag_world_msg = Float32MultiArray()
                tag_world_msg.data = [tag_world[0], tag_world[1], tag_world[2]+0.4, 0., 0., 0.5] # in order of xf and vf
            
                self.tag_world_pub.publish(tag_world_msg)
            
    def att_callback(self, msg):
        try:
            self.drone_q = msg.q
        except:
            self.get_logger().info("Oh no,,, att not received")

    def gps_callback(self, msg):
        try:
            self.drone_world = np.array([msg.x, msg.y, msg.z]) # NED frame
        except:
            self.get_logger().info("Oh no,,, position")
            

def main(args=None):
    rclpy.init(args=args)
    tagpublisher = TagPublisher()
    rclpy.spin(tagpublisher)

    tagpublisher.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
