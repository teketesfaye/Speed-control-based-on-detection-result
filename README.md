# Speed Controller with Object Detection based on Autoware

<div align="center">

![System Overview](images/FR_FL_Cameras.png)

</div>

A ROS2 package that integrates YOLOX object detection with Autoware trajectory control to create automatic stop commands when objects are detected in the vehicle's path.

## Overview

This package implements an autonomous safety system that:
- Uses forward-facing camera with YOLOX detection for pedestrians and objects
- Fuses detections from dual cameras to eliminate duplicates and improve accuracy
- Creates stop commands when objects are detected in front of the vehicle
- Stops the vehicle when pedestrians are detected on lane boundaries


## Key Features

- Object Detection: Real-time YOLOX detection of pedestrians, cars, and obstacles
- Dual Camera Fusion: Combines FL and FR camera detections with IoU-based overlap detection
- Automatic Safety Stop: Emergency stop when objects detected in trajectory path
- Pedestrian Safety: Immediate stop for pedestrians on lane boundaries
- Autoware Integration: Seamless integration with Autoware trajectory planning

### 1. Build Package
```bash
cd ~/autoware_ws
colcon build --packages-select speed_controller_cpp
source install/setup.bash
```

### 2. Single Camera Mode 
```bash
# Launch YOLOX detection
ros2 launch tensorrt_yolox yolox.launch.xml

# Launch speed controller
ros2 launch speed_controller_cpp speed_controller.launch.py
```

### 3. Dual Camera Mode 
```bash
# Terminal 1: FL camera
ros2 launch tensorrt_yolox yolox.launch.xml input_topic:=/sensing/camera/camera0/image_raw output_topic:=/perception/object_recognition/detection/rois0

# Terminal 2: FR camera
ros2 launch tensorrt_yolox yolox.launch.xml input_topic:=/sensing/camera/camera1/image_raw output_topic:=/perception/object_recognition/detection/rois1

# Terminal 3: Speed controller with fusion
ros2 launch speed_controller_cpp speed_controller.launch.py enable_dual_camera:=true
```
### Detection Fusion
- Single Object: Car detected by both cameras gets fused into one accurate detection
- Unique Objects: Objects seen by only one camera are preserved
- Result: Reduces false counts and improves detection accuracy

### Safety Scenarios
1. Object in Path: Vehicle stops when any object detected in trajectory
2. Pedestrian on Lane: Immediate stop for pedestrians near lane boundaries  
3. Emergency Override: Manual emergency stop always available

## Manual Control
```bash
# Set speed
ros2 topic pub /speed_command std_msgs/msg/Float64 "data: 5.0"

# Stop vehicle
ros2 topic pub /speed_command std_msgs/msg/Float64 "data: 0.0"

# Emergency stop
ros2 topic pub /speed_command std_msgs/msg/Float64 "data: -1.0"
```

## Dependencies
- ROS2 Humble
- Autoware.universe
- TensorRT YOLOX package
- OpenCV 4.x

## Architecture
```
Camera(s) → YOLOX Detection → Object Analysis → Speed Control → Autoware Trajectory
```
