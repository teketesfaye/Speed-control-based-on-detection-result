// IMplementation of autoware based object detection and control 
// Using yolox and autoware longitudinal and lateral control 
// Yolox detection,(ROS Topics and GUI Input for redundancy) ->Detection Analyzer -> Speed control PKG -> Trajectory modification -> Autoware COntroller excution 
// // Modify Autoware trajectories with speed profiles
// autoware_auto_planning_msgs::msg::Trajectory modifyTrajectorySpeed(
//     const autoware_auto_planning_msgs::msg::Trajectory& original_trajectory);


//  -Original Trajectory Generation
//    - Path Planning
//    - Behavior Planning  
//    - Motion Planning
// 
// - Speed Controller Integration Point
//    - Subscribes: /planning/scenario_planning/trajectory
//    - Processes: Speed Profile Injection on trajectory preserving all other attributes:position (x, y,orientation unchanged,lateral_velocity_mps unchanged,acceleration_mps2 unchanged,heading_rate_rps unchanged and wheel angles unchanged
//   - Publishes: /planning/scenario_planning/trajectory
// 
// - Vehicle Control Execution
//    - PID Controllers
//    - Actuator Commands
//    - Vehicle Response

#include "speed_controller_cpp/speed_controller_core.hpp"
#include <algorithm>
#include <cmath>

namespace speed_controller {

SpeedControllerCore::SpeedControllerCore(rclcpp::Node* node, 
                                       double speed_profile_rate, 
                                       double profile_update_frequency)
    : node_(node)
    , commanded_speed_(0.0)
    , current_speed_profile_(0.0)
    , target_speed_profile_(0.0)
    , has_received_speed_command_(false)
    , has_received_trajectory_(false)
    , speed_profile_rate_(speed_profile_rate)
    , profile_update_frequency_(profile_update_frequency)
    , emergency_applied_(false)
    , last_command_time_(node->get_clock()->now())
{
    RCLCPP_INFO(node_->get_logger(), "Speed Controller Core initialized");
    RCLCPP_INFO(node_->get_logger(), "  Speed profile rate: %.2f m/s per second", speed_profile_rate_);
    RCLCPP_INFO(node_->get_logger(), "  Update frequency: %.1f Hz", profile_update_frequency_);
    RCLCPP_INFO(node_->get_logger(), "  Maximum safe speed: %.1f m/s", MAX_SAFE_SPEED);
}

bool SpeedControllerCore::processSpeedCommand(double commanded_speed) {
    last_command_time_ = node_->get_clock()->now();
    
    if (!has_received_speed_command_) {
        has_received_speed_command_ = true;
        RCLCPP_INFO(node_->get_logger(), "SPEED PROFILE CONTROLLER ACTIVATED");
        RCLCPP_INFO(node_->get_logger(), "  Starting speed profile from: %.2f m/s", current_speed_profile_);
        RCLCPP_INFO(node_->get_logger(), "  Now injecting speed profiles into Autoware trajectory");
    }
    
    if (commanded_speed < 0.0) {
        RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP COMMAND RECEIVED");
        applyEmergencyStop();
        return true;
    }
    
    if (!validateSpeedCommand(commanded_speed)) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid speed command: %.2f m/s (exceeds limits)", commanded_speed);
        return false;
    }
    
    commanded_speed_ = commanded_speed;
    target_speed_profile_ = commanded_speed;
    emergency_applied_ = false;
    
    double transition_time = std::abs(target_speed_profile_ - current_speed_profile_) / speed_profile_rate_;
    
    RCLCPP_INFO(node_->get_logger(), "New speed profile target: %.2f m/s", commanded_speed);
    RCLCPP_INFO(node_->get_logger(), "  Smooth transition time: %.1f seconds", transition_time);
    RCLCPP_INFO(node_->get_logger(), "  Autoware PID will handle acceleration control");
    
    return true;
}

autoware_auto_planning_msgs::msg::Trajectory SpeedControllerCore::modifyTrajectorySpeed(
    const autoware_auto_planning_msgs::msg::Trajectory& original_trajectory) {
    
    auto modified_trajectory = original_trajectory;
    
    for (auto& point : modified_trajectory.points) {
        point.longitudinal_velocity_mps = static_cast<float>(current_speed_profile_);
    }
    
    modified_trajectory.header.stamp = node_->get_clock()->now();
    
    return modified_trajectory;
}

void SpeedControllerCore::updateSpeedProfile(double dt) {
    if (!shouldActivateControl()) {
        return;
    }
    
    double max_speed_change = speed_profile_rate_ * dt;
    double speed_difference = target_speed_profile_ - current_speed_profile_;
    
    if (std::abs(speed_difference) <= max_speed_change) {
        current_speed_profile_ = target_speed_profile_;
    } else {
        current_speed_profile_ += (speed_difference > 0) ? max_speed_change : -max_speed_change;
    }
    
    current_speed_profile_ = std::max(0.0, std::min(current_speed_profile_, MAX_SAFE_SPEED));
}

bool SpeedControllerCore::shouldActivateControl() const {
    return has_received_speed_command_ && has_received_trajectory_;
}

void SpeedControllerCore::applyEmergencyStop() {
    commanded_speed_ = 0.0;
    target_speed_profile_ = 0.0;
    current_speed_profile_ = 0.0;
    
    emergency_applied_ = true;
    
    RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP APPLIED");
    RCLCPP_WARN(node_->get_logger(), "  All speed profiles set to ZERO");
    RCLCPP_WARN(node_->get_logger(), "  Emergency flag ACTIVATED");
    RCLCPP_WARN(node_->get_logger(), "  Manual intervention required to resume operation");
}

void SpeedControllerCore::clearEmergencyStop() {
    emergency_applied_ = false;
    RCLCPP_INFO(node_->get_logger(), "Emergency stop cleared - Normal operation can resume");
}

bool SpeedControllerCore::validateSpeedCommand(double speed) const {
    if (speed < 0.0 && speed != EMERGENCY_STOP_SPEED) {
        return false;
    }
    
    if (speed > MAX_SAFE_SPEED) {
        return false;
    }
    
    return true;
}

void SpeedControllerCore::logControlStatus(bool detailed) const {
    if (detailed) {
        RCLCPP_INFO(node_->get_logger(), "=== SPEED CONTROL STATUS ===");
        RCLCPP_INFO(node_->get_logger(), "  Commanded Speed: %.2f m/s", commanded_speed_);
        RCLCPP_INFO(node_->get_logger(), "  Current Profile: %.2f m/s", current_speed_profile_);
        RCLCPP_INFO(node_->get_logger(), "  Target Profile: %.2f m/s", target_speed_profile_);
        RCLCPP_INFO(node_->get_logger(), "  Emergency Active: %s", emergency_applied_ ? "YES" : "NO");
        RCLCPP_INFO(node_->get_logger(), "  Control Active: %s", shouldActivateControl() ? "YES" : "NO");
        RCLCPP_INFO(node_->get_logger(), "============================");
    } else {
        std::string status_flag = emergency_applied_ ? "[EMERGENCY]" : "[NORMAL]";
        RCLCPP_INFO(node_->get_logger(), 
            "%s Speed: Cmd=%.1f Profile=%.1f m/s", 
            status_flag.c_str(), commanded_speed_, current_speed_profile_);
    }
}

} // namespace speed_controller