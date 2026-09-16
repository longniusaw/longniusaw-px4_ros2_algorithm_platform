#include <rclcpp/rclcpp.hpp>
#include "algorithm_platform/arbiter.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("arbiter"), "Arbiter node started (placeholder)");
  auto node = rclcpp::Node::make_shared("arbiter_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
