# Put the ros container you pulled from nvidia here
FROM dustynv/ros:iron-desktop-l4t-r35.2.1

# OpenVINS deps
RUN apt-get update && apt-get install -y libceres-dev

# tmux (quality of life)
RUN apt-get install -y tmux

# Source ROS install
RUN echo "source /opt/ros/iron/install/setup.bash" >> /root/.bashrc