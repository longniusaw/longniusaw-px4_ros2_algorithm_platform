#include "algorithm_platform/utils/timestamp.hpp"
#include <chrono>

namespace algorithm_platform {
namespace utils {

rclcpp::Time now() {
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

rclcpp::Time fromSeconds(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

double toSeconds(const rclcpp::Time& time) {
  return time.seconds();
}

} // namespace utils
} // namespace algorithm_platform
