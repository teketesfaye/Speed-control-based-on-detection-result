#ifndef SPEED_CONTROLLER_CPP__SPEED_CONTROLLER_HPP_
#define SPEED_CONTROLLER_CPP__SPEED_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

namespace speed_controller {

struct Detection {
    cv::Rect bbox;
    float confidence;
    std::string class_name;
    double distance;
    bool in_trajectory_path;
    int risk_level;
};

class DetectionConverter {
public:
    static std::vector<Detection> convertFromTensorRTYOLOX(
        const tier4_perception_msgs::msg::DetectedObjectsWithFeature& msg);
    static std::string getClassName(
        const autoware_auto_perception_msgs::msg::ObjectClassification& classification);
};

class SpeedController : public rclcpp::Node {
public:
    explicit SpeedController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SpeedController();

private:
    void speedCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
    void generateSpeedProfile();
    void updateSpeedProfile(double dt);
    bool shouldActivateControl() const;
    
    autoware_auto_planning_msgs::msg::Trajectory modifyTrajectorySpeed(
        const autoware_auto_planning_msgs::msg::Trajectory& original_traj, 
        double target_speed_profile);

    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void detectionCallback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr msg);
    void detectionCallbackFR(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr msg);
    std::vector<Detection> fuseDetections(const std::vector<Detection>& fl_dets, const std::vector<Detection>& fr_dets);
    double calculateIoU(const cv::Rect& box1, const cv::Rect& box2);

    void analyzeDetections();
    void checkObjectSafety();
    void countObjectsByPath();
    double calculateDistance(const Detection& det);
    bool isInTrajectoryPath(const cv::Point2f& point);
    
    void processDetections();

    void updateDisplay();
    void updateTrajectoryVisualization();
    void drawStatus(cv::Mat& image);
    void drawSafetyZones(cv::Mat& image);
    void drawTrajectoryPath(cv::Mat& image);
    void drawDetections(cv::Mat& image);
    void drawObjectCounts(cv::Mat& image);
    void drawGUI(cv::Mat& image);

    void initializeGUI();
    static void onMouseCallback(int event, int x, int y, int flags, void* userdata);
    void handleMouseEvent(int event, int x, int y, int flags);
    void increaseSpeed();
    void decreaseSpeed();
    void stopVehicle();
    void emergencyStopGUI();

    void processNativeStopSignBehavior();
    void publishStopSignDetection(const Detection& stop_sign_detection);
    void createStopLineMarker(const Detection& stop_sign_detection);

    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_output_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_;
    
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr objects_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stop_line_markers_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_command_sub_;
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_input_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr detection_sub_;
    rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr detection_sub_fr_;
    bool enable_dual_camera_;
    rclcpp::TimerBase::SharedPtr timer_;

    double commanded_speed_;
    double current_speed_profile_;
    double target_speed_profile_;
    bool has_received_speed_command_;
    bool has_received_trajectory_;
    double speed_profile_rate_;
    double profile_update_frequency_;
    
    rclcpp::Time last_update_time_;
    rclcpp::Time last_command_time_;
    rclcpp::Time stop_command_time_;
    int stop_failure_count_;
    
    bool emergency_applied_;
    bool object_emergency_active_;
    double closest_object_distance_;
    
    std::shared_ptr<autoware_auto_planning_msgs::msg::Trajectory> current_trajectory_;
    
    std::vector<Detection> current_detections_;
    std::vector<Detection> fl_detections_;
    std::vector<Detection> fr_detections_;
    std::mutex fr_detection_mutex_;
    cv::Mat current_image_;
    int frame_count_;
    int objects_in_path_;
    int safe_objects_;
    std::mutex detection_mutex_;
    
    double vehicle_speed_;
    cv::Point2f vehicle_position_;
    double vehicle_heading_;
    cv::Point2f reference_position_;
    bool reference_set_;
    
    std::vector<cv::Point2f> trajectory_points_image_;
    std::vector<cv::Point2f> left_lane_points_;
    std::vector<cv::Point2f> right_lane_points_;
    
    bool gui_enabled_;
    double gui_commanded_speed_;
    cv::Rect speed_up_button_;
    cv::Rect speed_down_button_;
    cv::Rect stop_button_;
    cv::Rect emergency_button_;
    std::chrono::steady_clock::time_point last_button_click_;
    
    double yaw_deviation_deg_;
    double lateral_deviation_m_;
    bool has_deviation_data_;
    
    static constexpr double SPEED_INCREMENT = 1.0;
    static constexpr double MAX_SPEED = 15.0;
    static constexpr double EMERGENCY_DISTANCE = 3.0;
    static constexpr double WARNING_DISTANCE = 6.0;
    static constexpr int DETECTION_SKIP = 2;
};

} 

#endif // 



