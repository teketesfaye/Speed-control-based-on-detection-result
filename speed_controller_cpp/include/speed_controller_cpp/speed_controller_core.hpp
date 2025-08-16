#ifndef SPEED_CONTROLLER_CPP__SPEED_CONTROLLER_CORE_HPP_
#define SPEED_CONTROLLER_CPP__SPEED_CONTROLLER_CORE_HPP_


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <chrono>

namespace speed_controller {

class SpeedControllerCore {
public:
    SpeedControllerCore(rclcpp::Node* node, 
                       double speed_profile_rate = 0.5, 
                       double profile_update_frequency = 10.0);
    
    bool processSpeedCommand(double commanded_speed);
    
    autoware_auto_planning_msgs::msg::Trajectory modifyTrajectorySpeed(
        const autoware_auto_planning_msgs::msg::Trajectory& original_trajectory);
    
    void updateSpeedProfile(double dt);
    
    bool shouldActivateControl() const;
    
    void setTrajectoryReceived(bool received) { has_received_trajectory_ = received; }
    
    void applyEmergencyStop();
    
    void clearEmergencyStop();
    
    double getCurrentSpeedProfile() const { return current_speed_profile_; }
    double getCommandedSpeed() const { return commanded_speed_; }
    double getTargetSpeedProfile() const { return target_speed_profile_; }
    bool isEmergencyActive() const { return emergency_applied_; }
    bool hasReceivedSpeedCommand() const { return has_received_speed_command_; }
    bool hasReceivedTrajectory() const { return has_received_trajectory_; }

private:
    rclcpp::Node* node_;
    
    double commanded_speed_;
    double current_speed_profile_;
    double target_speed_profile_;
    bool has_received_speed_command_;
    bool has_received_trajectory_;
    
    double speed_profile_rate_;
    double profile_update_frequency_;
    
    bool emergency_applied_;
    
    rclcpp::Time last_command_time_;
    
    bool validateSpeedCommand(double speed) const;
    
    void logControlStatus(bool detailed = false) const;
};

constexpr double MAX_SAFE_SPEED = 15.0;
constexpr double MIN_SPEED_INCREMENT = 0.1;
constexpr double EMERGENCY_STOP_SPEED = -1.0;

} 

#endif 