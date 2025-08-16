
#include "speed_controller_cpp/speed_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<speed_controller::SpeedController>(options);
  
  RCLCPP_INFO(node->get_logger(), "Speed Controller Node starting...");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in node: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}






