#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("fault_monitor"), "Fault monitor node started (placeholder)");
  auto node = rclcpp::Node::make_shared("fault_monitor_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
