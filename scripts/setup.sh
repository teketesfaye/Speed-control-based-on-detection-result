#!/bin/bash

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Build package
colcon build --packages-select speed_controller_cpp
source install/setup.bash

echo "Setup complete. Run: ros2 launch speed_controller_cpp speed_controller.launch.py"
