# Put the ros container you pulled from nvidia here
FROM dustynv/ros:iron-desktop-l4t-r35.2.1

# OpenVINS deps
RUN apt-get update && apt-get install -y libceres-dev

# Source ROS install
RUN echo "source /opt/ros/iron/install/setup.bash" >> /root/.bashrc