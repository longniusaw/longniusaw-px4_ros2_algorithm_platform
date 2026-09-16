#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "algorithm_platform/msg/trajectory_setpoint.hpp"

namespace algorithm_platform {

struct SetpointCommand {
  Eigen::Vector3d position;
  bool position_valid;
  Eigen::Vector3d velocity;
  bool velocity_valid;
  double yaw;
  bool yaw_valid;
  double yaw_rate;
  bool yaw_rate_valid;
  uint32_t sequence;
};

class Arbiter {
public:
  Arbiter();
  ~Arbiter() = default;

  void setGlobalPlan(const Eigen::Vector3d& goal);
  void setLocalCorrection(const Eigen::Vector3d& velocity_correction);
  void setCurrentPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
  void setObstacleDistance(double distance);
  
  SetpointCommand computeSetpoint();
  bool isSafetyCritical() const;

private:
  Eigen::Vector3d global_goal_;
  Eigen::Vector3d current_position_;
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d velocity_correction_;
  double min_obstacle_distance_;
  
  double safety_margin_;
  double max_velocity_;
  double max_acceleration_;
  double position_tolerance_;
  
  Eigen::Vector3d last_setpoint_;
  Eigen::Vector3d velocity_estimate_;
  
  SetpointCommand fuseCommands();
  void applySafetyLimits(SetpointCommand& cmd);
  void applySmoothFilter(SetpointCommand& cmd);
};

} // namespace algorithm_platform
