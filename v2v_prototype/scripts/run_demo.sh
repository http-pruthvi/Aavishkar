#!/bin/bash
# Build the workspace
colcon build --symlink-install

# Source the setup script
source install/setup.bash

# Run the launch file
ros2 launch v2v_prototype v2v.launch.py
