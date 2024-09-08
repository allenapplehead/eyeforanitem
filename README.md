# An Eye for an Item
Project Link: https://hackster.io/ceyeber/an-eye-for-an-item-9162a7

For instructions to set up this project, please go to the [Wiki](https://github.com/allenapplehead/eyeforanitem/wiki)

<a href="https://youtu.be/KAUH67vni74" target="_blank"><img src="https://img.youtube.com/vi/KAUH67vni74/0.jpg"></a>

## Repo Organization and Architecture Diagrams
To better make sense of the structure of this repository, please refer to the diagrams below:

### ROS2 Architecture
Everything in ros2 can be accessed by entering the docker container `./run_ros.sh`. The following diagram depicts the organization of all the components using ROS2:

![ros2_arch](https://github.com/allenapplehead/eyeforanitem/assets/44914805/624e5eab-baba-4859-8425-9148fc8fa03a)

### [`/arduino_code/drivebase`](/arduino_code/drivebase)
Contains vanilla arduino code for bluetooth teleop of the chassis. You can upload it through arduino IDE as a basic test that you've built and wired your chassis up correctly.

### [`/docker`](/docker)
Contains the Dockerfile based off of `dustynv/ros:humble-desktop-l4t-r35.2.1`, with all required dependencies for this specific project installed on top of it. Everything in this project can / should be run in this docker environment.

### [`/drivebase`](/drivebase)
Contains platformio project for ESP32, featuring IMU and motor control features, integrated through ROS to allow it to talk to the Jetson Orin via serial.

### [`/jetson-containers`](/jetson-containers)
Submodule. Fork of the [official jetson-containers project](https://github.com/dusty-nv/jetson-containers). Particular containers of interest include ROS2 Humble and Nanodb.

### [`/robot_ws/src`](/robot_ws/src)
ROS2 workspace for this project. See system architecture diagram for details.

### [`/scripts`](/scripts)
```
├── find_obj.py
└── visualize_tags.py
```
`find_obj.py` reads from the terminal outputs of nanodb after you issue a query for the object your finding, displays the best 8 matches, and visualizes the location of the robot when the selected image was taken. 

`visualize_tags.py` is a helper script that draws the map of the robot environment, apriltag locations, and of course the robot location where the image was taken. This script is called by `find_obj.py` and you don't need to run this script directly.


