#!/usr/bin/env bash

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Check for V4L2 devices
V4L2_DEVICES=""
for i in {0..9}; do
    if [ -e "/dev/video$i" ]; then
        V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
    fi
done

# Check for Arduino Uno
ARDUINO_DEVICE=""
if [ -e "/dev/ttyACM0" ]; then
    ARDUINO_DEVICE="--device /dev/ttyACM0"
fi

# Check for ESP32
ESP32_DEVICE=""
if [ -e "/dev/ttyUSB0" ]; then
    ESP32_DEVICE="--device /dev/ttyUSB0"
fi

# Check for display (X11 forwarding)
DISPLAY_DEVICE=""
if [ -n "$DISPLAY" ]; then
    sudo xhost +si:localuser:root

    XAUTH=/tmp/.docker.xauth
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod 777 $XAUTH

    DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
fi

# Check if sudo is needed
if id -nG "$USER" | grep -qw "docker"; then
    SUDO=""
else
    SUDO="sudo"
fi

# Run the container based on architecture
ARCH=$(uname -m)

if [ "$ARCH" = "aarch64" ]; then
    cat /proc/device-tree/model > /tmp/nv_jetson_model

    set -x
    $SUDO docker run --runtime nvidia -it --rm --network host \
        --volume /tmp/argus_socket:/tmp/argus_socket \
        --volume /etc/enctune.conf:/etc/enctune.conf \
        --volume /etc/nv_tegra_release:/etc/nv_tegra_release \
        --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
        --volume $ROOT/jetson-containers/data:/data \
        --device /dev/snd \
        --device /dev/bus/usb \
        $V4L2_DEVICES $DISPLAY_DEVICE $ARDUINO_DEVICE $ESP32_DEVICE \
        -v $(pwd):/workspace \
        -v /ssd/rosbags:/rosbags \
        --workdir /workspace \
        ${USER}/ros:iron-desktop-l4t-r35.2.1 "$@"

elif [ "$ARCH" = "x86_64" ]; then
    set -x
    $SUDO docker run --gpus all -it --rm --network=host \
        --shm-size=8g \
        --ulimit memlock=-1 \
        --ulimit stack=67108864 \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --volume $ROOT/jetson-containers/data:/data \
        $V4L2_DEVICES $DISPLAY_DEVICE $ARDUINO_DEVICE $ESP32_DEVICE \
        -v $(pwd):/workspace \
        -v /ssd/rosbags:/rosbags \
        --workdir /workspace \
        ${USER}/ros:iron-desktop-l4t-r35.2.1 "$@"
fi
