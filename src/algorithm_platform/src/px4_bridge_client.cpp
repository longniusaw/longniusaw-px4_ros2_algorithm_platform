#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("px4_bridge"), "PX4 bridge client started (placeholder)");
  auto node = rclcpp::Node::make_shared("px4_bridge_client");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
