#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("local_planner"), "Local planner node started (placeholder)");
  auto node = rclcpp::Node::make_shared("local_planner_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
