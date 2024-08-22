#!/bin/bash
cd ~/test_ws
source ./install/local_setup.bash

# Start USB camera node
echo "------------------------------"
echo "Starting USB camera node..."
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/test_ws/src/usb_cam/config/params_1.yaml &
sleep 5
echo "USB camera node started."

# Start YOLO detection
echo "------------------------------"
echo "Starting YOLO detection..."
ros2 run yolo_detection yolo_detector &
sleep 5
echo "YOLO detection started."

# Start Apriltag detection & Autolanding
echo "------------------------------"
echo "Starting Apriltag detection & Autolanding..."
ros2 launch goal_pub one.launch.py &
sleep 5
echo "Apriltag detection & Autolanding started."

# Start Vehicle controller
echo "------------------------------"
echo "Starting Vehicle controller..."
ros2 run test_nodes final_01 --ros-args --params-file ~/test_ws/src/vehicle_controller/config/final_01_hwasung.yaml
sleep 5
echo "Vehicle controller started."

# Keep the script running to maintain background processes
wait

