# Put the ros container you pulled from nvidia here
FROM dustynv/ros:humble-desktop-l4t-r35.2.1

# OpenVINS deps
RUN apt-get update && apt-get install -y libceres-dev

# tmux (quality of life)
RUN apt-get install -y tmux

# isaac ros apriltag deps
RUN apt-get install -y git-lfs

RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' /etc/apt/sources.list || \
    echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | tee -a /etc/apt/sources.list

RUN mkdir -p /etc/ros/rosdep/sources.list.d/ \
    && curl -o /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml https://isaac.download.nvidia.com/isaac-ros/extra_rosdeps.yaml \
    && echo "yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml" | tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list \
    && rosdep update

# tkinter deps
RUN apt-get install python3-pil python3-pil.imagetk -y

# Source ROS install
RUN echo "source /opt/ros/humble/install/setup.bash" >> /root/.bashrc