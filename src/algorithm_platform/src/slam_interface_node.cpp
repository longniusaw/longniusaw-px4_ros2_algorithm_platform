#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("slam_interface"), "SLAM interface node started (placeholder)");
  auto node = rclcpp::Node::make_shared("slam_interface_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
