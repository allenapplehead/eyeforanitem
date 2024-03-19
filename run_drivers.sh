#!/bin/bash

# Start a new tmux session and detach from it
tmux new-session -d -s imuCameraDrivers

# Split the window into two panes
tmux split-window -h

# Select the first pane (Pane 0)
tmux select-pane -t 0
# Send the commands to the first pane
tmux send-keys 'cd robot_ws' C-m
tmux send-keys 'source install/setup.bash' C-m
tmux send-keys 'ros2 run webcam_driver driver' C-m

# Select the second pane (Pane 1)
tmux select-pane -t 1
# Send the commands to the second pane
tmux send-keys 'cd microros_ws' C-m
tmux send-keys 'source install/setup.bash' C-m
tmux send-keys 'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0' C-m

# Attach to the tmux session
tmux attach-session -t imuCameraDrivers
