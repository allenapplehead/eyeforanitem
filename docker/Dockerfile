#
# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
FROM nvcr.io/nvidia/l4t-base:r35.2.1

ARG ROS_PACKAGE=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace


#
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-desktop-full && \
    apt-get install -y python3-rosdep && \
    rosdep init && \
    rosdep update && \
    apt-get install -y ros-$ROS_DISTRO-rospy && \
    apt-get install -y ros-${ROS_DISTRO}-rosserial-arduino && \
    apt-get install -y ros-${ROS_DISTRO}-rosserial

#
# install bootstrap dependencies
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          libpython3-dev \
          python3-rosinstall \
          python3-rosinstall-generator \
          python3-wstool \
          python3-vcstool \
          python3-catkin-tools \
          build-essential && \
    rm -rf /var/lib/apt/lists/*

# # /usr/bin/rosinstall_generator => No module named 'importlib.metadata'
# RUN pip3 install --no-cache-dir --verbose importlib.metadata


# #
# # download/build the ROS source
# # error: option --install-layout not recognized  https://github.com/ros/catkin/issues/863#issuecomment-1000446018
# #
# RUN mkdir ros_catkin_ws && \
#     cd ros_catkin_ws && \
#     rosinstall_generator ${ROS_PKG} vision_msgs image_transport --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \
#     mkdir src && \
#     vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
#     apt-get update && \
#     rosdep install -y \
# 	 --from-paths ./src \
# 	 --ignore-packages-from-source \
# 	 --rosdistro ${ROS_DISTRO} \
# 	 --skip-keys "python3-pykdl libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
#     python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF && \
#     rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

WORKDIR /