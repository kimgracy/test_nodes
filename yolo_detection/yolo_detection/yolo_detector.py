import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage as TfMsg
from my_bboxes_msg.msg import YoloObstacle, YoloTarget, VehiclePhase

import os
import sys
import time
import yaml

import cv2
import torch
import numpy as np
from cv_bridge import CvBridge


def is_jetson():
    try:
        with open('/etc/nv_tegra_release', 'r') as f:
            return True
    except FileNotFoundError:
        return False


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Variables
        self.phase = '8'
        self.subphase = 'yolo_only'
        self.y_threshold = 80
        self.frame_size = (1280, 720)
        self.yolo_size = (640, 360)


        # load camera info from yaml file
        self.camera_info = None
        self.camera_info_file = os.path.join(os.getcwd(), 'src/usb_cam/config/camera_info.yaml')
        if os.path.exists(self.camera_info_file):
            with open(self.camera_info_file, 'r') as f:
                self.camera_info = yaml.load(f, Loader=yaml.FullLoader)['camera_matrix']['data']
        print(f"Camera info: {self.camera_info}")


        # apriltag detection
        self.apriltag_detected = False
        self.apriltag_x = 0
        self.apriltag_y = 0


        # define model path and load the model
        if is_jetson():
            model_path = os.path.join(os.getcwd(), 'src/yolo_detection/config/best_small_50.pt')
            # model_path = os.path.join(os.getcwd(), 'src/yolo_detection/config/best_small_50.pt')
        else:
            print('Not run on Nvidia Jetson. use best_small.pt')
            model_path = os.path.join(os.getcwd(), 'src/yolo_detection/config/best_small_50.pt')            
        self.model = torch.hub.load(os.path.expanduser('~/yolov5'), 'custom', path=model_path, source='local')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        

        # Publishers
        self.publisher_obstacle = self.create_publisher(YoloObstacle, '/yolo_obstacle', qos_profile)
        # Subscribers
        self.vehicle_phase_subscriber = self.create_subscription(VehiclePhase, '/vehicle_phase', self.phase_callback, qos_profile)
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.apriltag_subscriber = self.create_subscription(TfMsg, 'tf', self.apriltag_callback, 10)
        

        # create target_capture folder, which is used to save target images
        self.target_capture_folder = os.path.join(os.getcwd(), 'src/yolo_detection/config/target_capture')
        os.makedirs(self.target_capture_folder, exist_ok=True)
        # timer for save target image
        self.timer_period = 0.2  # seconds
        self.last_capture_time = time.time()


        # create cv_bridge instance
        self.bridge = CvBridge()

        


    def image_callback(self, msg):
        # convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


        """
        YOLOv5 Detection
        (only when phase is 8)
        """
        if self.phase == '8' or self.phase == '7':
            # send frame to YOLOv5 model
            frame_resized = cv2.resize(frame, self.yolo_size)
            results = self.model(frame_resized)

            # extract bounding box, labels
            labels, cords = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

            # draw bounding box in frame & publish obstacle message
            # just use a bounding box with highest confidence
            if len(cords) > 0:
                row = cords[0]
                if row[4] >= 0.3:
                    x1, y1, x2, y2 = int(row[0] * frame.shape[1]), int(row[1] * frame.shape[0]), int(row[2] * frame.shape[1]), int(row[3] * frame.shape[0])
                    label = self.model.names[int(labels[0])]
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    
                    # publish obstacle message and draw bounding box
                    if (abs(y1-y2) >= self.y_threshold) and ((label == 'ladder-truck' or label == 'class4')):
                        # draw bounding box and label
                        label = 'ladder'
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f'{label} {row[4]:.2f}', (x1 + 5, y1 + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        
                        # publish obstacle message when phase is 8
                        #if (self.phase == '8'):
                        obstacle_msg = YoloObstacle()
                        obstacle_msg.label = label
                        obstacle_msg.x = x_center
                        obstacle_msg.y = y_center
                        self.publisher_obstacle.publish(obstacle_msg)


        """
        Save target image
        """
        if (self.phase == '3'):
            current_time = time.time()
            if current_time - self.last_capture_time >= self.timer_period:
                self.save_target_image(frame)
                self.last_capture_time = current_time


        """
        Display frame to monitor
        """
        # add phase information to the frame in black color
        cv2.putText(frame, f'Phase: {self.phase}', (10, 67), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3)
        cv2.putText(frame, f'Subphase: {self.subphase}', (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 3)


        """
        Display apriltag position
        """
        if self.apriltag_detected:
            cv2.circle(frame, (int(self.apriltag_x), int(self.apriltag_y)), 10, (0, 255, 0), -1)
            self.apriltag_detected = False

        
        # display frame to monitor
        cv2.imshow('YOLOv5 Detection', frame)

        # break whan keyinturrupt occurs    !!! DON'T REMOVE IT !!!
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
        


    def save_target_image(self, frame):
        timestamp = int(time.time())
        file_path = os.path.join(self.target_capture_folder, f'target_{timestamp}.jpg')
        cv2.imwrite(file_path, frame)
        # save the target image
        self.get_logger().info(f"Target image published and saved to {file_path}")
    
    

    def phase_callback(self, msg):
        # get vehicle phase
        self.phase = msg.phase
        self.subphase = msg.subphase


    def apriltag_callback(self, msg):
        # get apriltag position
        try:
            point_3d = msg.transforms[0].transform.translation
            self.apriltag_x = (self.camera_info[0] * point_3d.x / point_3d.z) + self.camera_info[2]
            self.apriltag_y = (self.camera_info[4] * point_3d.y / point_3d.z) + self.camera_info[5]
            self.apriltag_detected = True
        except:
            self.apriltag_detected = False



def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
