# eyeforanitem

`./run_ros.sh`: enter ros2 container

### ESP32 serial communication
```./run_ros.sh
source /opt/ros/iron/install/setup.bash
cd microros_ws
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
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
source /opt/ros/iron/install/setup.bash
cd robot_ws
source install/local_setup.bash
ros2 launch image_collector image_collector_launch.py
```
Saves the images to `.../eyeforanitem/jetson-containers/data/datasets/image_collector/...`

### Teleoperation
If you wish to drive the robot manually:
* Upload `arduino_code/drivebase/drivebase.ino` onto the ESP32
* Download an arduino car app on your phone. I'm using "Arduino Car"
* On your phone, connect to ESP32_BT_Car
* Set the controls to match the character commands in the uploaded arduino script
