# An Eye for an Item

### Build ROS2 container
```
docker pull dustynv/ros:humble-desktop-l4t-r35.2.1
cd docker
docker build -t ${USER}/ros:humble-desktop-l4t-r35.2.1 .
```

### Run ESP32 serial communication, IMU, and Camera Drivers
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
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh /ssd/eyeforanitem/isaac_ros-dev
sudo apt-get install -y ros-humble-isaac-ros-apriltag
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

### Teleoperation
If you wish to drive the robot manually:
* Upload `arduino_code/drivebase/drivebase.ino` onto the ESP32
* Download an arduino car app on your phone. I'm using "Arduino Car"
* On your phone, connect to ESP32_BT_Car
* Set the controls to match the character commands in the uploaded arduino script

