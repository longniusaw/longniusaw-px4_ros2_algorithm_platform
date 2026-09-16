#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("global_planner"), "Global planner node started (placeholder)");
  auto node = rclcpp::Node::make_shared("global_planner_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
