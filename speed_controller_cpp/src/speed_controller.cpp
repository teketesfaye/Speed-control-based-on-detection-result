
 
// Report toDaniel Tobias
// Fuse the detections together if there is overlap, otherwise add the detections to a single topic in 2d space

//  1. Detection Flow

//   Single Camera:
//   FL Camera -> YOLOX -> rois0 -> detectionCallback -> current_detections

//   Dual Camera Mode:
//   FL Camera -> YOLOX -> rois0 -> detectionCallback -> fl_detections_
//   FR Camera -> YOLOX -> rois1 -> detectionCallbackFR -> fr_detections_
//   fuseDetections() ->
//    current_detections

//   2. Fusion Algorithm

//   The fuseDetections() :

//   Step 1: Find Overlaps
//   - For each FL detection, check all FR detections
//   - Calculate IoU (Intersection over Union) between bounding boxes
//   - If IoU > 0.3 (30% overlap), consider them as the same object

//   Step 2: Fuse Overlapping Detections
//   // Average the bounding box coordinates
//   fused_x = (fl_bbox.x + fr_bbox.x) / 2
//   fused_y = (fl_bbox.y + fr_bbox.y) / 2
//   fused_width = (fl_bbox.width + fr_bbox.width) / 2
//   fused_height = (fl_bbox.height + fr_bbox.height) / 2

//   // Average confidence and distance
//   fused_confidence = (fl_confidence + fr_confidence) / 2
//   fused_distance = (fl_distance + fr_distance) / 2

//   Step 3: Add Non-overlapping Detections
//   - Add all FL detections 
//   - Add FR detections that had no overlap with FL detections

//   3. IoU Calculation

//   IoU = Intersection_Area / Union_Area

//   Where:
//   Intersection_Area = overlapping rectangle area
//   Union_Area = total area covered by both boxes

//   Example:
//   - FL detects car at [100, 100, 50, 80]
//   - FR detects same car at [110, 105, 55, 75]
//   - IoU = 0.45 (45% overlap)
//   - Since 0.45 > 0.3, they get fused to [105, 102, 52, 77]

//   4. Result
//   Without Fusion:
//   - FL: car, motorcycle, truck
//   - FR: car, bicycle, truck
//   - Total: 6 separate detections

//   With Fusion:
//   - Same car,truck detected by both: get averaged
//   - Total: 4 detections (1 fused car + 1 motorcycle + bicycle + 1 fused truck )

// By merging overlapping detections from both cameras, the dual camera detection fusion reduces unnecessary object counts and increases detection accuracy. 
// By averaging their positions and confidence scores, the fusion algorithm reduces false counts from six detections (three per camera with duplicates) 
// to four unique objects (one fused car + one motorcycle + one bicycle + one fused truck), avoiding overly conservative speed control decisions 
// while preserving full field of view coverage

#include "speed_controller_cpp/speed_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

namespace speed_controller {


constexpr double WARNING_DISTANCE = 6.0;

std::vector<Detection> DetectionConverter::convertFromTensorRTYOLOX(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature& msg) {
    
    std::vector<Detection> detections;
    
    for (const auto& obj : msg.feature_objects) {
        Detection det;
        
        det.bbox = cv::Rect(
            static_cast<int>(obj.feature.roi.x_offset),
            static_cast<int>(obj.feature.roi.y_offset),
            static_cast<int>(obj.feature.roi.width),
            static_cast<int>(obj.feature.roi.height)
        );
        
        det.confidence = obj.object.existence_probability;
        
        det.class_name = getClassName(obj.object.classification[0]);
        
        det.distance = 0.0;
        det.in_trajectory_path = false;
        det.risk_level = 0;
        
        detections.push_back(det);
    }
    
    return detections;
}

std::string DetectionConverter::getClassName(
    const autoware_auto_perception_msgs::msg::ObjectClassification& classification) {
    
    switch (classification.label) {
        case autoware_auto_perception_msgs::msg::ObjectClassification::CAR:
            return "car";
        case autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK:
            return "truck";
        case autoware_auto_perception_msgs::msg::ObjectClassification::BUS:
            return "bus";
        case autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER:
            return "truck";
        case autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
            return "motorcycle";
        case autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE:
            return "bicycle";
        case autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
            return "person";
        default:
            return "unknown";
    }
}
SpeedController::SpeedController(const rclcpp::NodeOptions & options)
: Node("speed_profile_controller", options), 
  commanded_speed_(0.0), 
  current_speed_profile_(0.0),
  target_speed_profile_(0.0),
  has_received_speed_command_(false),
  has_received_trajectory_(false),
  speed_profile_rate_(0.5),
  profile_update_frequency_(10.0),
  stop_command_time_(rclcpp::Time(0)),
  stop_failure_count_(0),
  emergency_applied_(false),
  object_emergency_active_(false),
  closest_object_distance_(100.0),
  frame_count_(0),
  objects_in_path_(0),
  safe_objects_(0),
  vehicle_speed_(0.0),
  vehicle_position_(cv::Point2f(0.0, 0.0)),
  vehicle_heading_(0.0),
  reference_position_(cv::Point2f(0.0, 0.0)),
  reference_set_(false),
  gui_enabled_(true),
  gui_commanded_speed_(0.0),
  last_button_click_(std::chrono::steady_clock::now()),
  yaw_deviation_deg_(0.0),
  lateral_deviation_m_(0.0),
  has_deviation_data_(false),
  enable_dual_camera_(false)
{
  this->declare_parameter("speed_profile_rate", 0.5);
  this->declare_parameter("profile_update_frequency", 10.0);
  this->declare_parameter("trajectory_input_topic", "/planning/scenario_planning/trajectory");
  this->declare_parameter("trajectory_output_topic", "/planning/scenario_planning/trajectory");
  this->declare_parameter("enable_dual_camera", false);
  
  speed_profile_rate_ = this->get_parameter("speed_profile_rate").as_double();
  profile_update_frequency_ = this->get_parameter("profile_update_frequency").as_double();
  enable_dual_camera_ = this->get_parameter("enable_dual_camera").as_bool();
  
  std::string trajectory_input_topic = this->get_parameter("trajectory_input_topic").as_string();
  std::string trajectory_output_topic = this->get_parameter("trajectory_output_topic").as_string();

  speed_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/speed_command", 
    rclcpp::QoS(10),
    std::bind(&SpeedController::speedCommandCallback, this, std::placeholders::_1)
  );

  trajectory_input_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory_original",
    rclcpp::QoS(10),
    std::bind(&SpeedController::trajectoryCallback, this, std::placeholders::_1)
  );

  trajectory_output_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory",
    rclcpp::QoS(10)
  );

  emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/system/emergency/control_cmd",
    rclcpp::QoS(10)
  );

  debug_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/debug/speed_profile_controller",
    rclcpp::QoS(10)
  );

  objects_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "/perception/object_recognition/objects",
    rclcpp::QoS(10)
  );
  
  stop_line_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/scenario_planning/stop_line_markers",
    rclcpp::QoS(10)
  );

  auto qos = rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/sensing/camera/camera0/image_raw", qos,
    std::bind(&SpeedController::cameraCallback, this, std::placeholders::_1)
  );
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", qos,
    std::bind(&SpeedController::odomCallback, this, std::placeholders::_1)
  );

  // YOLOX detection for FL camera (rois0)
  detection_sub_ = this->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "/perception/object_recognition/detection/rois0", qos,
    std::bind(&SpeedController::detectionCallback, this, std::placeholders::_1)
  );
  
  // Optional FR camera (rois1)
  if (enable_dual_camera_) {
    detection_sub_fr_ = this->create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "/perception/object_recognition/detection/rois1", qos,
      std::bind(&SpeedController::detectionCallbackFR, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Dual camera mode enabled - subscribing to rois1");
  } else {
    RCLCPP_INFO(this->get_logger(), "Single camera mode - FL only (rois0)");
  }

  double timer_period = 1.0 / profile_update_frequency_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period), 
    std::bind(&SpeedController::generateSpeedProfile, this)
  );

  last_update_time_ = this->get_clock()->now();
  last_command_time_ = this->get_clock()->now();

  initializeGUI();

  RCLCPP_INFO(this->get_logger(), "Speed Profile Controller Package - ENHANCED");
  RCLCPP_INFO(this->get_logger(), "Started with TensorRT YOLOX Detection + GUI");
  RCLCPP_INFO(this->get_logger(), "   Speed profile rate: %.2f m/s per second", speed_profile_rate_);
  RCLCPP_INFO(this->get_logger(), "   Update frequency: %.1f Hz", profile_update_frequency_);
  RCLCPP_INFO(this->get_logger(), "   Detection: Waiting for TensorRT YOLOX node...");
  RCLCPP_INFO(this->get_logger(), " Terminal Commands:");
  RCLCPP_INFO(this->get_logger(), "   ros2 topic pub /speed_command std_msgs/msg/Float64 \"data: 4.0\"");
  RCLCPP_INFO(this->get_logger(), "   ros2 topic pub /speed_command std_msgs/msg/Float64 \"data: 0.0\"");
  RCLCPP_INFO(this->get_logger(), "   ros2 topic pub /speed_command std_msgs/msg/Float64 \"data: -1.0\"");
  RCLCPP_INFO(this->get_logger(), " GUI: Use mouse to click speed control buttons");
}

SpeedController::~SpeedController() {
}

void SpeedController::speedCommandCallback(const std_msgs::msg::Float64::SharedPtr msg) 
{
  last_command_time_ = this->get_clock()->now();

  if (!has_received_speed_command_) {
    has_received_speed_command_ = true;
    if (current_trajectory_ && !current_trajectory_->points.empty()) {
      current_speed_profile_ = current_trajectory_->points[0].longitudinal_velocity_mps;
    }
    RCLCPP_INFO(this->get_logger(), " SPEED PROFILE CONTROLLER ACTIVATED");
    RCLCPP_INFO(this->get_logger(), "   Starting speed profile from: %.2f m/s", current_speed_profile_);
    RCLCPP_INFO(this->get_logger(), "   Now injecting speed profiles into Autoware trajectory");
  }

  if (msg->data < 0.0) {
    RCLCPP_WARN(this->get_logger(), " EMERGENCY STOP");
    commanded_speed_ = 0.0;
    target_speed_profile_ = 0.0;
    current_speed_profile_ = 0.0;
    emergency_applied_ = true;
    
    auto emergency_msg = std_msgs::msg::Bool();
    emergency_msg.data = true;
    emergency_pub_->publish(emergency_msg);
    
    gui_commanded_speed_ = 0.0;
    return;
  }

  commanded_speed_ = msg->data;
  target_speed_profile_ = commanded_speed_;
  emergency_applied_ = false;
  
  gui_commanded_speed_ = commanded_speed_;
  
  RCLCPP_INFO(this->get_logger(), " New speed profile target: %.2f m/s", commanded_speed_);
  
  double transition_time = std::abs(target_speed_profile_ - current_speed_profile_) / speed_profile_rate_;
  RCLCPP_INFO(this->get_logger(), "   Smooth transition time: %.1f seconds", transition_time);
  RCLCPP_INFO(this->get_logger(), "   Autoware PID will handle acceleration control");
}

void SpeedController::trajectoryCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) 
{
  current_trajectory_ = msg;
  
  if (!has_received_trajectory_) {
    has_received_trajectory_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to Autoware trajectory stream");
    RCLCPP_INFO(this->get_logger(), "Trajectory points: %zu", msg->points.size());
    if (!msg->points.empty()) {
      RCLCPP_INFO(this->get_logger(), "Original speed: %.2f m/s", 
                  msg->points[0].longitudinal_velocity_mps);
      if (!has_received_speed_command_) {
        current_speed_profile_ = msg->points[0].longitudinal_velocity_mps;
      }
    }
  }

  updateTrajectoryVisualization();

  bool has_stop_behavior = false;
  for (const auto& point : msg->points) {
    if (point.longitudinal_velocity_mps <= 0.01) {
      has_stop_behavior = true;
      break;
    }
  }

  if (has_stop_behavior && !emergency_applied_ && commanded_speed_ != 0.0) {
    trajectory_output_pub_->publish(*msg);
    return;
  }

  if (!shouldActivateControl()) {
    trajectory_output_pub_->publish(*msg);
    return;
  }

  auto modified_trajectory = modifyTrajectorySpeed(*msg, current_speed_profile_);
  trajectory_output_pub_->publish(modified_trajectory);
}

bool SpeedController::shouldActivateControl() const
{
  return has_received_speed_command_ && has_received_trajectory_;
}

void SpeedController::updateSpeedProfile(double dt)
{
  if (!shouldActivateControl()) {
    return;
  }

  if (target_speed_profile_ == 0.0 || emergency_applied_) {
    current_speed_profile_ = 0.0;
    return;
  }

  double max_speed_change = speed_profile_rate_ * dt;
  double speed_diff = target_speed_profile_ - current_speed_profile_;
  
  if (std::abs(speed_diff) <= max_speed_change) {
    current_speed_profile_ = target_speed_profile_;
  } else {
    current_speed_profile_ += (speed_diff > 0) ? max_speed_change : -max_speed_change;
  }
}

// Preserve all other attributes:position (x, y,orientation unchanged,lateral_velocity_mps unchanged,acceleration_mps2 unchanged,heading_rate_rps unchanged and wheel angles unchanged
// While changeing longitudinal_velocity_mps 
autoware_auto_planning_msgs::msg::Trajectory SpeedController::modifyTrajectorySpeed(
  const autoware_auto_planning_msgs::msg::Trajectory& original_traj, 
  double target_speed_profile)
{
  auto modified_traj = original_traj;
  
  for (auto& point : modified_traj.points) {
    point.longitudinal_velocity_mps = static_cast<float>(target_speed_profile);
  }
  
  modified_traj.header.stamp = this->get_clock()->now();
  
  return modified_traj;
}

void SpeedController::generateSpeedProfile()
{
  auto now = this->get_clock()->now();
  double dt = (now - last_update_time_).seconds();
  last_update_time_ = now;
  
  if (dt > 0.2) {
    dt = 1.0 / profile_update_frequency_;
  }

  updateSpeedProfile(dt);

  auto debug_msg = std_msgs::msg::Float64();
  debug_msg.data = current_speed_profile_;
  debug_pub_->publish(debug_msg);


  static int log_count = 0;
  if (shouldActivateControl() && log_count % 20 == 0) {
    std::string status_info = emergency_applied_ ? "[EMERGENCY_STOP]" : "[NORMAL]";
    RCLCPP_INFO(this->get_logger(), 
      " %s Speed Profile: Commanded=%.1f Profile=%.1f m/s | Publishing trajectory to Autoware", 
      status_info.c_str(), commanded_speed_, current_speed_profile_);
      
    if (emergency_applied_ && current_speed_profile_ != 0.0) {
        RCLCPP_ERROR(this->get_logger(), "BUG: Emergency stop but speed profile not 0! Forcing to 0.");
        current_speed_profile_ = 0.0;
    }
  }
  log_count++;
}

void SpeedController::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        current_image_ = cv_ptr->image.clone();
        
        // Update GUI display with current image
        updateDisplay();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Camera callback error: %s", e.what());
    }
}

void SpeedController::detectionCallback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr msg) {
    try {
        std::vector<Detection> new_detections = DetectionConverter::convertFromTensorRTYOLOX(*msg);
        
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            fl_detections_ = new_detections;
            
            if (enable_dual_camera_) {
                // Fuse FL and FR detections
                std::lock_guard<std::mutex> fr_lock(fr_detection_mutex_);
                current_detections_ = fuseDetections(fl_detections_, fr_detections_);
            } else {
                // Single camera mode - use FL only
                current_detections_ = std::move(new_detections);
            }
        }
        
        analyzeDetections();
        countObjectsByPath();
        processDetections();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "FL detection callback error: %s", e.what());
    }
}

void SpeedController::detectionCallbackFR(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr msg) {
    try {
        std::vector<Detection> new_detections = DetectionConverter::convertFromTensorRTYOLOX(*msg);
        
        {
            std::lock_guard<std::mutex> lock(fr_detection_mutex_);
            fr_detections_ = new_detections;
            
            // Fuse with FL detections
            std::lock_guard<std::mutex> fl_lock(detection_mutex_);
            current_detections_ = fuseDetections(fl_detections_, fr_detections_);
        }
        
        analyzeDetections();
        countObjectsByPath();
        processDetections();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "FR detection callback error: %s", e.what());
    }
}

void SpeedController::countObjectsByPath() {
    objects_in_path_ = 0;
    safe_objects_ = 0;
    
    for (const auto& det : current_detections_) {
        if (det.in_trajectory_path) {
            objects_in_path_++;
        } else {
            safe_objects_++;
        }
    }
}

void SpeedController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    vehicle_speed_ = std::sqrt(
        msg->twist.twist.linear.x * msg->twist.twist.linear.x +
        msg->twist.twist.linear.y * msg->twist.twist.linear.y
    );
    
    vehicle_position_.x = msg->pose.pose.position.x;
    vehicle_position_.y = msg->pose.pose.position.y;
    
    auto& q = msg->pose.pose.orientation;
    vehicle_heading_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    
    if (!reference_set_) {
        reference_position_ = vehicle_position_;
        reference_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Reference position set for trajectory visualization");
    }
}

void SpeedController::updateTrajectoryVisualization() {
    if (!current_trajectory_ || !reference_set_ || current_trajectory_->points.empty()) {
        return;
    }
    
    trajectory_points_image_.clear();
    left_lane_points_.clear();
    right_lane_points_.clear();
    
    const int img_width = 2048;
    const int img_height = 1536;
    const int img_center_x = img_width / 2;
    const int display_lane_width_pixels = 1000;
    
    float steering_curve_factor = 0.0f;
    float lateral_offset_pixels = 0.0f;
    
    
    const int start_y = img_height - 80;
    const int end_y = static_cast<int>(img_height * 0.35);
    const int bottom_width = display_lane_width_pixels;
    const int top_width = display_lane_width_pixels / 5;
    
    for (int y = start_y; y > end_y; y -= 25) {
        double progress = static_cast<double>(start_y - y) / static_cast<double>(start_y - end_y);
        
        float distance_factor = (start_y - y) * 0.008f;
        float curve_offset = steering_curve_factor * distance_factor;
        curve_offset = std::max(-120.0f, std::min(120.0f, curve_offset));
        
        float center_x = img_center_x + curve_offset + lateral_offset_pixels * (1.0f - progress * 0.5f);
        center_x = std::max(static_cast<float>(img_width * 0.2), 
                           std::min(static_cast<float>(img_width * 0.8), center_x));
        
        int current_width = bottom_width - static_cast<int>((bottom_width - top_width) * progress);
        int half_width = current_width / 2;
        
        cv::Point2f left_point(center_x - half_width, y);
        cv::Point2f right_point(center_x + half_width, y);
        
        trajectory_points_image_.push_back(cv::Point2f(center_x, y));
        left_lane_points_.push_back(left_point);
        right_lane_points_.push_back(right_point);
    }
}

void SpeedController::analyzeDetections() {
    closest_object_distance_ = 100.0;
    
    for (auto& det : current_detections_) {
        det.distance = calculateDistance(det);
        
        cv::Point2f center(det.bbox.x + det.bbox.width/2.0f, 
                          det.bbox.y + det.bbox.height/2.0f);
        det.in_trajectory_path = isInTrajectoryPath(center);
        
        if (det.in_trajectory_path) {
            if (det.distance <= EMERGENCY_DISTANCE) {
                det.risk_level = 2;
            } else if (det.distance <= WARNING_DISTANCE) {
                det.risk_level = 1;
            } else {
                det.risk_level = 0;
            }
            
            closest_object_distance_ = std::min(closest_object_distance_, det.distance);
        }
    }
}

void SpeedController::checkObjectSafety() {
    if (current_detections_.empty()) {
        object_emergency_active_ = false;
        return;
    }
    
    bool emergency_detected = false;
    bool warning_detected = false;
    std::string closest_object_name;
    
    for (const auto& det : current_detections_) {
        if (det.in_trajectory_path && det.risk_level == 2) {
            emergency_detected = true;
            closest_object_name = det.class_name;
            break;
        } else if (det.in_trajectory_path && det.risk_level == 1) {
            warning_detected = true;
            if (closest_object_name.empty()) closest_object_name = det.class_name;
        }
    }
    
    if (emergency_detected) {
        object_emergency_active_ = true;
        static int warning_count = 0;
        if (warning_count % 50 == 0) {
            RCLCPP_ERROR(this->get_logger(), 
                "OBJECT EMERGENCY: %s at %.1fm - MANUAL STOP REQUIRED", 
                closest_object_name.c_str(), closest_object_distance_);
        }
        warning_count++;
    } else if (warning_detected) {
        object_emergency_active_ = false;
        static int warning_count = 0;
        if (warning_count % 100 == 0) {
            RCLCPP_WARN(this->get_logger(), 
                "OBJECT WARNING: %s at %.1fm - CAUTION ADVISED", 
                closest_object_name.c_str(), closest_object_distance_);
        }
        warning_count++;
    } else {
        object_emergency_active_ = false;
    }
}

double SpeedController::calculateDistance(const Detection& det) {
    if (det.bbox.height > 20) {
        return (600.0 * 1.5) / det.bbox.height;
    }
    return 50.0;
}

bool SpeedController::isInTrajectoryPath(const cv::Point2f& point) {
    if (trajectory_points_image_.empty()) {
        int image_center_x = current_image_.cols / 2;
        int lane_width = 400;
        return std::abs(point.x - image_center_x) < lane_width / 2;
    }
    
    double min_distance_to_path = 1000.0;
    
    for (const auto& traj_point : trajectory_points_image_) {
        double distance = std::sqrt(
            (point.x - traj_point.x) * (point.x - traj_point.x) +
            (point.y - traj_point.y) * (point.y - traj_point.y)
        );
        min_distance_to_path = std::min(min_distance_to_path, distance);
    }
    
    return min_distance_to_path < 120.0;
}

void SpeedController::initializeGUI() {
    cv::namedWindow("Speed Controller", cv::WINDOW_NORMAL);
    cv::resizeWindow("Speed Controller", 1200, 700);  // Wider window to fit buttons
    cv::setMouseCallback("Speed Controller", onMouseCallback, this);
    
    int button_width = 200;
    int button_height = 50;
    int start_x = 1650;
    int start_y = 20;
    
    speed_up_button_ = cv::Rect(start_x, start_y, button_width, button_height);
    speed_down_button_ = cv::Rect(start_x, start_y + 80, button_width, button_height);
    stop_button_ = cv::Rect(start_x, start_y + 160, button_width, button_height);
    emergency_button_ = cv::Rect(start_x, start_y + 240, button_width, button_height);
    
    RCLCPP_INFO(this->get_logger(), "DEBUG: speed_down_button_ = (%d, %d, %d, %d)", 
                speed_down_button_.x, speed_down_button_.y, speed_down_button_.width, speed_down_button_.height);
    
    RCLCPP_INFO(this->get_logger(), " CRITICAL GUI - TOP RIGHT POSITION with EXTRA LARGE BUTTONS");
}

void SpeedController::updateDisplay() {
    if (current_image_.empty()) return;
    
    cv::Mat display = current_image_.clone();
    
    drawSafetyZones(display);
    drawTrajectoryPath(display);
    drawDetections(display);
    drawObjectCounts(display);
    drawStatus(display);
    drawGUI(display);
    
    cv::imshow("Speed Controller", display);
    cv::waitKey(1);
}

void SpeedController::onMouseCallback(int event, int x, int y, int flags, void* userdata) {
    SpeedController* controller = static_cast<SpeedController*>(userdata);
    controller->handleMouseEvent(event, x, y, flags);
}

void SpeedController::handleMouseEvent(int event, int x, int y, int flags) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    
    auto now = std::chrono::steady_clock::now();
    auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_button_click_).count();
    
    if (time_since_last < 200) return;
    
    last_button_click_ = now;
    
    cv::Point click_point(x, y);
    
    if (speed_up_button_.contains(click_point)) {
        increaseSpeed();
    } else if (speed_down_button_.contains(click_point)) {
        decreaseSpeed();
    } else if (stop_button_.contains(click_point)) {
        stopVehicle();
    } else if (emergency_button_.contains(click_point)) {
        emergencyStopGUI();
    }
}

void SpeedController::increaseSpeed() {
    if (emergency_applied_) {
        RCLCPP_WARN(this->get_logger(), "GUI: Cannot increase speed - Emergency stop active");
        return;
    }
    
    gui_commanded_speed_ += SPEED_INCREMENT;
    if (gui_commanded_speed_ > MAX_SPEED) {
        gui_commanded_speed_ = MAX_SPEED;
    }
    
    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = gui_commanded_speed_;
    speedCommandCallback(speed_msg);
    
    RCLCPP_INFO(this->get_logger(), "GUI: Speed increased to %.1f m/s", gui_commanded_speed_);
}

void SpeedController::decreaseSpeed() {
    if (emergency_applied_) {
        RCLCPP_WARN(this->get_logger(), "GUI: Cannot decrease speed - Emergency stop active");
        return;
    }
    
    gui_commanded_speed_ -= SPEED_INCREMENT;
    if (gui_commanded_speed_ < 0.0) {
        gui_commanded_speed_ = 0.0;
    }
    
    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = gui_commanded_speed_;
    speedCommandCallback(speed_msg);
    
    RCLCPP_INFO(this->get_logger(), "GUI: Speed decreased to %.1f m/s", gui_commanded_speed_);
}

void SpeedController::stopVehicle() {
    gui_commanded_speed_ = 0.0;
    
    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = 0.0;
    speedCommandCallback(speed_msg);
    
    RCLCPP_WARN(this->get_logger(), "GUI: STOP commanded");
}

void SpeedController::emergencyStopGUI() {
    gui_commanded_speed_ = 0.0;
    
    auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
    speed_msg->data = -1.0;
    speedCommandCallback(speed_msg);
    
    RCLCPP_ERROR(this->get_logger(), "GUI: EMERGENCY STOP commanded");
}

void SpeedController::processDetections() {
    if (commanded_speed_ == 0.0 || emergency_applied_) {
        return;
    }
    
    bool critical_object_detected = false;
    std::string object_info;
    double closest_distance = 100.0;
    
    for (const auto& det : current_detections_) {
        if (det.in_trajectory_path && det.distance < WARNING_DISTANCE) {
            critical_object_detected = true;
            if (det.distance < closest_distance) {
                closest_distance = det.distance;
                object_info = det.class_name + " at " + std::to_string(static_cast<int>(det.distance)) + "m";
            }
        }
    }
    
    if (critical_object_detected) {
        auto speed_msg = std::make_shared<std_msgs::msg::Float64>();
        speed_msg->data = -1.0;
        speedCommandCallback(speed_msg);
        
        RCLCPP_ERROR(this->get_logger(), "DETECTION: EMERGENCY STOP - %s", object_info.c_str());
    }
}

void SpeedController::drawSafetyZones(cv::Mat& image) {
    const int img_height = image.rows;
    const int img_width = image.cols;
    
    int red_line_y = static_cast<int>(img_height * 0.73);
    cv::line(image, cv::Point(0, red_line_y), cv::Point(img_width, red_line_y), 
             cv::Scalar(0, 0, 255), 6);
    cv::putText(image, "EMERGENCY STOP ZONE (3m)", cv::Point(50, red_line_y - 10), 
               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    
    int yellow_line_y = static_cast<int>(img_height * 0.52);
    cv::line(image, cv::Point(0, yellow_line_y), cv::Point(img_width, yellow_line_y), 
             cv::Scalar(0, 255, 255), 4);
    cv::putText(image, "WARNING ZONE (6m)", cv::Point(50, yellow_line_y - 10), 
               cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
}

// 2-lane trajectory visual
void SpeedController::drawTrajectoryPath(cv::Mat& image) {
    int img_width = image.cols;
    int img_height = image.rows;
    int center_x = img_width / 2;
    
    if (trajectory_points_image_.size() > 1) {
        for (size_t i = 1; i < trajectory_points_image_.size(); ++i) {
            cv::line(image, trajectory_points_image_[i-1], trajectory_points_image_[i], 
                    cv::Scalar(255, 255, 255), 6);
        }
    }
    
    if (left_lane_points_.size() > 1 && right_lane_points_.size() > 1) {
        for (size_t i = 1; i < left_lane_points_.size(); ++i) {
            cv::line(image, left_lane_points_[i-1], left_lane_points_[i], 
                    cv::Scalar(0, 255, 0), 4);
        }
        
        for (size_t i = 1; i < right_lane_points_.size(); ++i) {
            cv::line(image, right_lane_points_[i-1], right_lane_points_[i], 
                    cv::Scalar(255, 0, 0), 4);
        }
    }
    
    int ego_x = center_x;
    int ego_y = img_height - 80;
    
    cv::Rect vehicle_rect(ego_x - 25, ego_y - 50, 50, 100);
    cv::rectangle(image, vehicle_rect, cv::Scalar(255, 255, 255), 3);
    cv::putText(image, "EGO", cv::Point(ego_x - 15, ego_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
}

void SpeedController::drawDetections(cv::Mat& image) {
    for (const auto& det : current_detections_) {
        cv::Scalar color;
        int thickness = 5;
        
        if (det.in_trajectory_path) {
            color = cv::Scalar(0, 0, 255);
        } else if (det.distance <= WARNING_DISTANCE) {
            color = cv::Scalar(0, 255, 255);
        } else {
            color = cv::Scalar(0, 255, 0);
        }
        
        cv::rectangle(image, det.bbox, color, thickness);
        
        std::string label = det.class_name + " " + std::to_string(static_cast<int>(det.distance)) + "m";
        if (det.in_trajectory_path) {
            label += " [PATH]";
        }
        
        cv::putText(image, label, cv::Point(det.bbox.x, det.bbox.y - 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }
}

void SpeedController::drawObjectCounts(cv::Mat& image) {
    std::string count_text = "Objects in path: " + std::to_string(objects_in_path_) + 
                           " | Safe objects: " + std::to_string(safe_objects_);
    
    cv::Size text_size = cv::getTextSize(count_text, cv::FONT_HERSHEY_SIMPLEX, 1.3, 3, nullptr);
    int x_pos = (image.cols - text_size.width) / 2;
    
    cv::Rect text_bg(x_pos - 15, 200, text_size.width + 30, text_size.height + 25);
    cv::rectangle(image, text_bg, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(image, text_bg, cv::Scalar(255, 255, 255), 3);
    
    cv::Scalar text_color;
    if (objects_in_path_ > 0) {
        text_color = cv::Scalar(0, 140, 255);
    } else {
        text_color = cv::Scalar(0, 255, 0);
    }
    
    cv::putText(image, count_text, cv::Point(x_pos, 230), 
               cv::FONT_HERSHEY_SIMPLEX, 1.3, text_color, 3);
}

void SpeedController::drawStatus(cv::Mat& image) {
    cv::rectangle(image, cv::Rect(5, 5, 550, 120), cv::Scalar(0, 0, 0), -1);
    cv::rectangle(image, cv::Rect(5, 5, 550, 120), cv::Scalar(255, 255, 255), 2);
    
    std::string speed_text = "Current Speed: " + std::to_string(static_cast<int>(current_speed_profile_ * 3.6)) + " km/h";
    cv::putText(image, speed_text, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    
    std::string command_text = "Commanded: " + std::to_string(static_cast<int>(commanded_speed_ * 3.6)) + " km/h";
    cv::putText(image, command_text, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    
    std::string status_text;
    cv::Scalar status_color;
    
    if (emergency_applied_) {
        status_text = "STATUS: EMERGENCY STOP ACTIVE";
        status_color = cv::Scalar(0, 0, 255);
    } else if (object_emergency_active_) {
        status_text = "STATUS: OBJECT IN DANGER ZONE";
        status_color = cv::Scalar(0, 165, 255);
    } else if (commanded_speed_ == 0.0) {
        status_text = "STATUS: STOPPED";
        status_color = cv::Scalar(0, 255, 255);
    } else {
        status_text = "STATUS: NORMAL OPERATION";
        status_color = cv::Scalar(0, 255, 0);
    }
    
    cv::putText(image, status_text, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2);
    
    std::string control_text = "Speed Control: ";
    if (!shouldActivateControl()) {
        control_text += "INACTIVE";
        cv::putText(image, control_text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 2);
    } else {
        control_text += "ACTIVE";
        cv::putText(image, control_text, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
    
    if (!current_detections_.empty()) {
        std::string obj_text = "Objects: " + std::to_string(current_detections_.size()) + 
                              " | In Path: " + std::to_string(objects_in_path_);
        cv::putText(image, obj_text, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 2);
    }
}

void SpeedController::drawGUI(cv::Mat& image) {
    cv::rectangle(image, speed_up_button_, cv::Scalar(0, 255, 0), -1);
    cv::rectangle(image, speed_down_button_, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(image, stop_button_, cv::Scalar(0, 165, 255), -1);
    cv::rectangle(image, emergency_button_, cv::Scalar(0, 0, 255), -1);
    
    cv::rectangle(image, speed_up_button_, cv::Scalar(255, 255, 255), 4);
    cv::rectangle(image, speed_down_button_, cv::Scalar(255, 255, 255), 4);
    cv::rectangle(image, stop_button_, cv::Scalar(255, 255, 255), 4);
    cv::rectangle(image, emergency_button_, cv::Scalar(255, 255, 255), 4);
    
    cv::putText(image, "SPEED +", 
               cv::Point(speed_up_button_.x + 25, speed_up_button_.y + 35), 
               cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 0), 3);
    cv::putText(image, "SPEED -", 
               cv::Point(speed_down_button_.x + 25, speed_down_button_.y + 35), 
               cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    cv::putText(image, "STOP", 
               cv::Point(stop_button_.x + 50, stop_button_.y + 35), 
               cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 3);
    cv::putText(image, "EMERGENCY", 
               cv::Point(emergency_button_.x + 10, emergency_button_.y + 35), 
               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3);
}



void SpeedController::processNativeStopSignBehavior() {
    
    for (const auto& det : current_detections_) {
        if (det.class_name == "stop sign" && det.in_trajectory_path) {
            publishStopSignDetection(det);
            
            createStopLineMarker(det);
            
            RCLCPP_INFO(this->get_logger(), 
                "NATIVE AUTOWARE: Stop sign detected at %.1fm - Published to behavior planner", 
                det.distance);
        }
    }
}

void SpeedController::publishStopSignDetection(const Detection& stop_sign_detection) {
    auto objects_msg = autoware_auto_perception_msgs::msg::PredictedObjects();
    objects_msg.header.stamp = this->get_clock()->now();
    objects_msg.header.frame_id = "base_link";
    
    autoware_auto_perception_msgs::msg::PredictedObject obj;
    
    autoware_auto_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
    classification.probability = stop_sign_detection.confidence;
    obj.classification.push_back(classification);
    
    obj.kinematics.initial_pose_with_covariance.pose.position.x = stop_sign_detection.distance;
    obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
    
    obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    obj.kinematics.initial_pose_with_covariance.pose.orientation.x = 0.0;
    obj.kinematics.initial_pose_with_covariance.pose.orientation.y = 0.0;
    obj.kinematics.initial_pose_with_covariance.pose.orientation.z = 0.0;
    
    obj.shape.type = autoware_auto_perception_msgs::msg::Shape::POLYGON;
    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = 0.5; p1.y = 0.5; p1.z = 0.0;
    p2.x = -0.5; p2.y = 0.5; p2.z = 0.0;
    p3.x = -0.5; p3.y = -0.5; p3.z = 0.0;
    p4.x = 0.5; p4.y = -0.5; p4.z = 0.0;
    obj.shape.footprint.points = {p1, p2, p3, p4};
    
    obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
    obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
    obj.kinematics.initial_twist_with_covariance.twist.linear.z = 0.0;
    
    obj.object_id.uuid[0] = 's';
    obj.object_id.uuid[1] = 't';
    obj.object_id.uuid[2] = 'o';
    obj.object_id.uuid[3] = 'p';
    
    objects_msg.objects.push_back(obj);
    
    objects_pub_->publish(objects_msg);
}

void SpeedController::createStopLineMarker(const Detection& stop_sign_detection) {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "stop_signs";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = stop_sign_detection.distance;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 1.0;
    
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.1;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    marker.lifetime = rclcpp::Duration::from_seconds(2.0);
    
    marker_array.markers.push_back(marker);
    
    stop_line_markers_pub_->publish(marker_array);
}

// Detection Fusion Implementation
std::vector<Detection> SpeedController::fuseDetections(const std::vector<Detection>& fl_dets, 
                                                       const std::vector<Detection>& fr_dets) {
    std::vector<Detection> fused_detections;
    std::vector<bool> fr_used(fr_dets.size(), false);
    
    // Process FL detections and find overlaps with FR
    for (const auto& fl_det : fl_dets) {
        bool found_overlap = false;
        Detection fused_det = fl_det;
        
        for (size_t i = 0; i < fr_dets.size(); ++i) {
            if (fr_used[i]) continue;
            
            double iou = calculateIoU(fl_det.bbox, fr_dets[i].bbox);
            if (iou > 0.3) { // 30% overlap threshold
                // Fuse the detections
                int fused_x = (fl_det.bbox.x + fr_dets[i].bbox.x) / 2;
                int fused_y = (fl_det.bbox.y + fr_dets[i].bbox.y) / 2;
                int fused_w = (fl_det.bbox.width + fr_dets[i].bbox.width) / 2;
                int fused_h = (fl_det.bbox.height + fr_dets[i].bbox.height) / 2;
                
                fused_det.bbox = cv::Rect(fused_x, fused_y, fused_w, fused_h);
                fused_det.confidence = (fl_det.confidence + fr_dets[i].confidence) / 2.0f;
                fused_det.distance = (fl_det.distance + fr_dets[i].distance) / 2.0;
                
                fr_used[i] = true;
                found_overlap = true;
                break;
            }
        }
        
        fused_detections.push_back(fused_det);
    }
    
    // Add non-overlapping FR detections
    for (size_t i = 0; i < fr_dets.size(); ++i) {
        if (!fr_used[i]) {
            fused_detections.push_back(fr_dets[i]);
        }
    }
    
    return fused_detections;
}

double SpeedController::calculateIoU(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);
    
    if (x2 <= x1 || y2 <= y1) {
        return 0.0;
    }
    
    int intersection = (x2 - x1) * (y2 - y1);
    int union_area = box1.area() + box2.area() - intersection;
    
    return static_cast<double>(intersection) / union_area;
}

} // namespace speed_controller



