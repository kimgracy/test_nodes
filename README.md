# PX4-Autopilot

대회에 사용할 PX4-Autopilot **v1.15.0-beta2**를 설치해야 한다.  
기존의 PX4-Autopilot에서 바로 checkout 해도 되지만, 혹시 모를 issue에 대비해서 그냥 지웠다가 다시 까는 것을 추천한다. ([reference](https://docs.px4.io/main/en/contribute/git_examples.html#get-a-specific-release))

1. 기존 PX4-Autopilot 제거

   ```
   cd
   rm -rf PX4-Autopilot/
   ```

2. PX4-Autopilot clone

   ```
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd PX4-Autopilot/
   ```

3. version 변경

   ```
   git checkout v1.15.0-beta2
   ```

4. submodules update (오래 걸림)

   ```
   make submodulesclean
   ```

5. Ubuntu 환경 setup & 컴퓨터 재부팅  
   (보통 안 해도 잘 되긴 하는데 재부팅 안 했다고 오류 찾느라 고생한 적이 있다. 혹시 모르니 아래 명령어 실행 후 그냥 재부팅을 하도록 하자.)

   ```
   bash ./Tools/setup/ubuntu.sh
   ```

6. build (오래 걸림)

   ```
   cd PX4-Autopilot/
   make px4_sitl gazebo-classic
   ```

7. 소프트웨어팀 Notion에서 bulnabi.world를 download 받은 후 적혀있는 방식 따라서 world 추가

8. standard_vtol도 download 받은 후 압축 풀어서 PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models 경로에 덮어씌우기

9. 대회 환경 build

   ```
   cd PX4-Autopilot/
   make px4_sitl gazebo-classic_standard_vtol__bulnabi
   ```

10. PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml 에서 아래 내용들을 각각 publications와 subscriptions 항목의 가장 마지막 부분에 추가
    ```
    - topic: /fmu/out/vtol_vehicle_status
    type: px4_msgs::msg::VtolVehicleStatus
    ```
    ```
    - topic: /fmu/in/gimbal_manager_set_manual_control
    type: px4_msgs::msg::GimbalManagerSetManualControl
    ```

# Installation

1. 해당 repo fork  
   scroll을 위로 올려보면 Edit pins / Watch / Fork / Star 라고 적힌 부분이 있는데 Fork를 클릭하고 Create Fork를 눌러주자.

2. workspace 만들기

   ```
   cd
   mkdir test_ws
   cd test_ws
   ```

3. 본인 repository를 보면 fork된 repo가 생겼을텐데, 해당 repo clone

   ```
   git clone https://github.com/[your_github_id]/test_nodes.git
   ```

4. src 폴더로 이름 변경

   ```
   mv test_nodes src
   ```

5. directory 구조 확인

   ```
   tree -aL 2
   ```

   (정상적인 출력 결과)

   ```
   └── src
       ├── .git
       ├── my_bboxes_msg
       ├── px4_msgs
       ├── README.md
       ├── vehicle_controller
       └── yolo_detection

   6 directories, 1 file
   ```

6. upstream 등록

   ```
   cd src
   git remote add upstream https://github.com/Bulnabi-SNU/test_nodes.git
   ```

7. git 연결 확인

   ```
   git remote -v
   ```

   (정상적인 출력 결과)

   ```
    origin	https://github.com/[your_github_id]/test_nodes.git (fetch)
    origin	https://github.com/[your_github_id]/test_nodes.git (push)
    upstream	https://github.com/Bulnabi-SNU/test_nodes.git (fetch)
    upstream	https://github.com/Bulnabi-SNU/test_nodes.git (push)
   ```

8. (Optional) alias 추가  
   나는 잘 안 하는 편인데 원하는 사람은 .bashrc에 `alias rosfoxy='source ~/test_ws/install/local_setup.bash'`와 같이 추가해도 좋다.

# Requirements

1. Install v4l2_camera

   ```
   sudo apt update
   sudo apt install ros-${ROS_DISTRO}-v4l2-camera
   ```

   <br/>

2. Download yolov5 at home directory

   ```
   cd
   git clone https://github.com/ultralytics/yolov5
   ```

   <br/>

3. (Optional) v4l2-ctl: change v4l2_camera settings  
   (WSL을 사용하는 경우에는 안 되기 때문에 pass.. 혹시라도 되면 알려주면 감사 ㅜ)

   ```
   sudo apt install v4l-utils

   v4l2-ctl -d /dev/video0 --list-formats-ex   # camera formats 확인
   v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480   # change settings
   ```

# Build

- 아래와 같이 colcon build 한번만 실행해도 되는걸로 변경했는데 잘 돌아가는지 채원아 확인 부탁...
- 해당 repo에 아직 auto landing 관련 코드는 없기 때문에 auto landing은 dependency 잘 고려해서 build 문제 없도록 코드 올리기!

```
cd test_ws
colcon build --symlink-install                  // cba
source ./install/local_setup.bash
```

# Development

### Warning

- px4_msgs는 별도로 clone 받을 필요 없이 해당 repo에 있는 것 기준으로 코드를 작성하면 된다.
- sitl과 달리 실제 비행에서는 기체의 시작 위치의 local position이 origin이 아니다. 따라서 특정 WP로 이동하려면 takeoff 할 때 home position의 위치를 설정한 후, 상대적인 위치를 계산해야 한다. (test_02에서 self.home_position 관련 코드 확인)
- 코드를 실행하더라도 기체가 바로 동작하지 않고, RC에서 스위치를 켜서 허용해야지 동작하도록 코드를 작성해야 한다. RC에서 offboard mode로 변환하는 스위치가 있기 때문에 offboard mode인지 확인 후 takeoff 하는 코드를 추가하면 된다. (test_02 takeoff_and_arm_callback 관련 코드 확인)
- 본인이 맡은 part의 코드 추가 후, sitl에서 잘 되는지 여러 번 검증하고 기체의 움직임이 너무 과하지 않은지 확인해야 한다. (이동할 때 roll, pitch 등이 너무 큰 폭으로 변하지 않는지 검증) sitl에서조차 기체의 움직임이 과격하다면 실제 비행에서는 더 큰 문제가 발생할 수 있다.
- 비행 시험의 편의성을 위해서 여러 개의 node를 run 해야 하는 경우에는 하나의 launch file로 만드는 것을 추천한다.

### Code Upload

1. README.md 수정  
   본인의 test 코드 실행 명령어 및 간략한 설명을 아래의 Run 부분에 추가

2. upstream 변경 사항 반영
   ```
   git pull upstream main
   ```
3. 충돌 문제가 발생한다면, 충돌 문제 해결 후 병

4. 수정 사항을 본인 repo에 올리기

   ```
   git add .
   git commit -m "[YOUR_COMMIT_MESSAGE]" # e.g. FIX: yolo_test_00 gimbal control
   git push origin main
   ```

5. 본인 repo에 들어가서 commit이 제대로 됐는지 확인한다. Pull requests에 들어가서 New pull request를 클릭한다. 본인이 어떤 코드를 작성했는지 **자세하게** 설명하고 reviewer에 okj001010을 추가 후, Create pull request 버튼을 클릭한다.

6. 여기까지 한 이후에는 코드 확인 후 직접 merge 할 예정이니, (아마 안 되긴 할텐데) **절대** 바로 merge 버튼을 누르지 말아야 한다.

# Run

터미널 6개를 실행해야 한다.

terminal 1:
```
MicroXRCEAgent udp4 -p 8888
```

terminal 2:
```
./QGroundControl.AppImage
```

terminal 3:
```
cd PX4-Autopilot/
make px4_sitl gazebo-classic_standard_vtol__bulnabi
```

terminal 4:
```
ros2 run v4l2_camera v4l2_camera_node
```

terminal 5:
```
cd test_ws/
source ./install/local_setup.bash   (rosfoxy)
ros2 run yolo_detection yolo_detector 
```

terminal 6:
```
cd test_ws/
source ./install/local_setup.bash   (rosfoxy)
ros2 run test_nodes mc_test_00
ros2 run test_nodes mc_test_01
ros2 run test_nodes mc_test_02 --ros-args --params-file ~/test_ws/src/vehicle_controller/config/mc_test_02_waypoint.yaml
ros2 run test_nodes yolo_test_01
```

- mc_test_00: arming
- mc_test_01: takeoff → land
- mc_test_02: takeoff → N 2m → NE 2m → E 2m → home → land (사각 비행)
- yolo_test_01: takeoff → gimbal control → ladder detection → land

<br/>
<br/>
<br/>

# Streaming Images with MJPG-streamer

- Start MJPG-streamer before execute launch files

- Streaming website: http://127.0.0.1:8080/?action=stream

```
# install mjpg streamer
git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
make
sudo make install
```

```
# run mjpg streamer
cd mjpg-streamer/mjpg-streamer-experimental
./mjpg_streamer -i "./input_file.so -f /tmp -n stream.jpg -d 0.1" -o "./output_http.so -w ./www"
```
