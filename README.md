# An Eye for an Item

### Build ROS2 container
```
docker pull dustynv/ros:humble-desktop-l4t-r35.2.1
cd docker
docker build -t ${USER}/ros:humble-desktop-l4t-r35.2.1 .
```

### Run ESP32 serial communication, IMU, and teleop drivers
```
./run_ros.sh  # enter docker container
./run_drivers.sh
```

### NanoDB commands
```cd jetson-containers
./run.sh -v ${PWD}/data/datasets/image_collector/train:/my_dataset $(./autotag nanodb) \
  python3 -m nanodb \
    --scan /my_dataset \
    --path /my_dataset/nanodb \
    --autosave --validate
```

```cd jetson-containers
./run.sh -v ${PWD}/data/datasets/image_collector/train:/my_dataset $(./autotag nanodb) \
  python3 -m nanodb \
    --path /my_dataset/nanodb \
    --server --port=7860
```

### Run data collection
If you are visualizing through vscode ssh, start vcxsrv
```cd eyeforanitem
./run_ros.sh
source /opt/ros/humble/install/setup.bash
cd robot_ws
source install/local_setup.bash
ros2 launch image_collector image_collector_launch.py
```
Saves the images to `.../eyeforanitem/jetson-containers/data/datasets/image_collector/...`

### Run 2D localization
```
./run_ros.sh
In one terminal, ./run_drivers.sh
In another terminal, rviz2. Select the loc.rviz config file in .../robot_ws/src/localizer/config/
In a third terminal: cd robot_ws, source install/setup.bash, ros2 run localizer localizer
```

### Isaac ROS Apriltags
```
set ISAAC_ROS_WS environment variable

# in one terminal
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh /ssd/eyeforanitem/isaac_ros-dev
sudo apt-get install -y ros-humble-isaac-ros-apriltag
cd /workspaces/isaac_ros-dev && \
  colcon build --symlink-install && \
  source install/setup.bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_usb_cam.launch.py

# in a second terminal
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh /ssd/eyeforanitem/isaac_ros-dev
rviz2 -d /workspaces/isaac_ros-dev/src/isaac_ros_apriltag/isaac_ros_apriltag/rviz/usb_cam.rviz
```

### Record a rosbag (for offline processing)
```
./run_ros.sh
source robot_ws/install/setup.bash
cd /rosbags  # or where-ever you put your rosbags
ros2 bag record -o tag_test /camera_info /camera_info/nitros /camera_info_rect /camera_info_rect/nitros /drivebase_subscriber /image_raw /image_raw/compressed /image_raw/compressedDepth /image_raw/nitros /image_raw/theora /image_rect /image_rect/nitros /image_rect/nitros/nitros_image_rgb8 /imu /tag_detections /tag_detections/nitros
```

### Run 2D Odometry
```
./run_ros.sh
source robot_ws/install/setup.bash
ros2 launch localizer localizer_launch.py

# if you wish to preview this, in a second terminal:
./run_ros.sh
rviz2
open loc.rviz in localizer package
```

### Teleoperation
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

If you instead just wish to drive the robot manually using pure arduino code without any Jetson software:
* Upload `arduino_code/drivebase/drivebase.ino` onto the ESP32
* Download an arduino car app on your phone. I'm using "Arduino Car"
* On your phone, connect to ESP32_BT_Car
* Set the controls to match the character commands in the uploaded arduino script

