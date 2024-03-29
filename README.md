# An Eye for an Item
Project Link: https://hackster.io/ceyeber/an-eye-for-an-item-9162a7

## 1. Setup

### 1.1 Build ROS2 containers
Due to Docker images being large in nature, you are basically required to buy and attach an SSD to your Jetson Orin. Then, you can set it up by following this guide: [https://www.jetson-ai-lab.com/tips_ssd-docker.html](https://www.jetson-ai-lab.com/tips_ssd-docker.html)

Then, build the ros2 humble container:
```
docker pull dustynv/ros:humble-desktop-l4t-r35.2.1
cd docker
docker build -t ${USER}/ros:humble-desktop-l4t-r35.2.1 .
```

Next, follow instructions on: https://www.jetson-ai-lab.com/tutorial_nanodb.html to build nanodb docker container. You don't need the COCO dataset.

### 1.2 Setup ESP32
The Jetson Orin is connected to an ESP32 via microusb to interface with our IMU and DC motors.

This [guide](https://www.youtube.com/watch?v=Nf7HP9y6Ovo) is really good to set up Platformio to build your own ESP32 scripts, and set up ESP32 for ros2 integration. Just follow this Youtube guide exactly but use the files in `/drivebase` instead of his starter code.

<mark>After successful setup, you should have `/microros_ws` on the root level of this repository</mark>

### 1.3 Setup Isaac ROS (with CUDA accelerated Apriltags)

Complete these 4 guides in this order:
* https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html
* https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart
Assuming you are using a USB, monocular camera:
* https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/camera_calibration.html
* https://nvidia-isaac-ros.github.io/concepts/fiducials/apriltag/tutorial_usb_cam.html
If you are using a different camera, follow the corresponding Isaac ROS guide for your hardware

<mark>After successful setup, you should have `/isaac_ros-dev` on the root level of this repository. You should also add `export ISAAC_ROS_WS=/ssd/eyeforanitem/isaac_ros-dev/` to your `~/.bashrc` file.
</mark>

## 2. Run whole pipeline on Robot

## 2.1 Data Collection

You will have to spin up several terminals (I recommend `terminator` on Ubuntu for a clean way to manage several terminals at once)

### 2.1.1 Run ESP32 serial communication, IMU, and teleop drivers
```
./run_ros.sh  # enter docker container
./run_drivers.sh
```

### 2.1.2 Run usb-camera driver and Isaac ROS Apriltag Detections
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh /ssd/eyeforanitem/isaac_ros-dev
sudo apt-get install -y ros-humble-isaac-ros-apriltag
cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py
```
If you want to visualize detections, run:
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh /ssd/eyeforanitem/isaac_ros-dev
rviz2 -d /workspaces/isaac_ros-dev/src/isaac_ros_apriltag/isaac_ros_apriltag/rviz/usb_cam.rviz
```

### 2.1.3 Run 2D localization
```
./run_ros.sh
source robot_ws/install/setup.bash
ros2 launch localizer localizer_launch.py
```
If you wish to view fused 2D odometry results (between model based and apriltag based localization):
```
./run_ros.sh
rviz2
open loc.rviz in localizer package
```

### 2.1.4 Teleoperation
```
# in one terminal
./run_ros.sh
./run_drivers.sh

# in a second terminal
./run_ros.sh
cd robot_ws
source install/setup.bash
ros2 run localizer teleop_keyboard
```

### 2.1.5 Start Collecting Images
If you are visualizing through vscode ssh, start vcxsrv
```cd eyeforanitem
./run_ros.sh
source /opt/ros/humble/install/setup.bash
cd robot_ws
source install/local_setup.bash
ros2 launch image_collector image_collector_launch.py
```
Saves the images to `.../eyeforanitem/jetson-containers/data/datasets/image_collector/...`

## 2.2 Post-processing and finding your item through NanoDB

### 2.2.1 NanoDB commands
If you've just collected your dataset, you need to build your embeddings first. This only needs to be once per data collection
```cd jetson-containers
./run.sh -v ${PWD}/data/datasets/image_collector/train:/my_dataset $(./autotag nanodb) \
  python3 -m nanodb \
    --scan /my_dataset \
    --path /my_dataset/nanodb \
    --autosave --validate
```

To spin up the gradio webserver (allows text or image queries) and the command line query environment to enter your missing object, run this:
```
cd /ssd/eyeforanitem/jetson-containers && ./run.sh -v ${PWD}/data/datasets/image_collector/train:/my_dataset $(./autotag nanodb) \
  python3 -m nanodb \
    --path /my_dataset/nanodb \
    --server --port=7860 --k 8 | tee /ssd/eyeforanitem/scripts/out.txt
```

To process your most recent command line query (saved into `out.txt` by previous command), and get location stamps for where the image was taken, run this:
```
./run_ros.sh
cd scripts
python3 find_obj.py
```

## 3. Bonus / Helpful features

### Record a rosbag (for offline processing)
This is helpful as you can just run `2.1.1` and `2.1.2` (necessary drivers) as well as `2.1.5` to drive the robot around, save everything the robot sees, and perform the rest of the steps by replaying back this data through `ros2 bag play <name of your bag>`
```
./run_ros.sh
source robot_ws/install/setup.bash
cd /rosbags  # or where-ever you put your rosbags
ros2 bag record -o tag_test /drivebase_subscriber /image_raw /imu /tag_detections
```

If you instead just wish to drive the robot manually using pure arduino code without any Jetson software:
* Upload `arduino_code/drivebase/drivebase.ino` onto the ESP32
* Download an arduino car app on your phone. I'm using "Arduino Car"
* On your phone, connect to ESP32_BT_Car
* Set the controls to match the character commands in the uploaded arduino script

## 4. Architecture Diagrams
To better make sense of the structure of this repository, please refer to the diagrams below:

### ROS2 Architecture
![ros2_arch](https://github.com/allenapplehead/eyeforanitem/assets/44914805/624e5eab-baba-4859-8425-9148fc8fa03a)
