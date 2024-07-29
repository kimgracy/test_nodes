#!/bin/bash

# Start Micro XRCE-DDS Agent
echo "------------------------------"
echo "Starting Micro XRCE-DDS Agent..."
sudo MicroXRCEAgent serial --dev /dev/ttyTHS0 -b 921600 &
sleep 5
echo "Micro XRCE-DDS Agent started."

# Start MJPG Streamer
echo "------------------------------"
echo "Starting MJPG Streamer..."
cd ~/mjpg-streamer/mjpg-streamer-experimental
./mjpg_streamer -i "./input_file.so -f /tmp -n stream.jpg -d 0.1" -o "./output_http.so -w ./www" &
sleep 5
echo "MJPG Streamer started."

cd ~/test_ws
source ./install/local_setup.bash

# Start USB camera node
echo "------------------------------"
echo "Starting USB camera node..."
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ~/test_ws/src/usb_cam/config/params_1.yaml &
sleep 5
echo "USB camera node started."

# Sart YOLO detection
echo "------------------------------"
echo "Starting YOLO detection..."
ros2 run yolo_detection yolo_detector &
sleep 5
echo "YOLO detection started."

# Keep the script running to maintain background processes
wait

