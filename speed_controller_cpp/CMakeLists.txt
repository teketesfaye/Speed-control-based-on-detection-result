cmake_minimum_required(VERSION 3.8)
project(speed_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

# NEW: Object Detection dependencies
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

# TensorRT YOLOX dependency (replaces ONNX Runtime)
find_package(tier4_perception_msgs REQUIRED)

# NATIVE AUTOWARE INTEGRATION dependencies
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(speed_controller_node 
  src/speed_controller_node.cpp
  src/speed_controller.cpp
  src/speed_controller_core.cpp
)

# Target include directories
target_include_directories(speed_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(speed_controller_node
  rclcpp
  std_msgs
  autoware_auto_planning_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  OpenCV
  autoware_auto_perception_msgs
  tier4_perception_msgs
  visualization_msgs
  geometry_msgs
)

# Link libraries
target_link_libraries(speed_controller_node 
  ${OpenCV_LIBS}
)

# Set C++ standard
target_compile_features(speed_controller_node PUBLIC cxx_std_17)

# Install targets
install(TARGETS speed_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()