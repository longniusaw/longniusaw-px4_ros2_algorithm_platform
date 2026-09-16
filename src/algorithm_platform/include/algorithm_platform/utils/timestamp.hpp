#pragma once
#include <rclcpp/rclcpp.hpp>

namespace algorithm_platform {
namespace utils {

rclcpp::Time now();
rclcpp::Time fromSeconds(double seconds);
double toSeconds(const rclcpp::Time& time);

} // namespace utils
} // namespace algorithm_platform
