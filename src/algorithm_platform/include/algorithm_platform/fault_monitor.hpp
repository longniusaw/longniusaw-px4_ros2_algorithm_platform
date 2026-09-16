#pragma once
#include <rclcpp/rclcpp.hpp>
#include "algorithm_platform/msg/algorithm_heartbeat.hpp"

namespace algorithm_platform {

struct ModuleHealth {
  bool slam_healthy;
  bool global_planner_healthy;
  bool local_planner_healthy;
  bool arbiter_healthy;
  bool communication_healthy;
  rclcpp::Time last_update;
};

class FaultMonitor {
public:
  FaultMonitor();
  ~FaultMonitor() = default;

  void updateModuleHealth(const ModuleHealth& health);
  uint8_t getOverallStatus() const;
  uint8_t getFaultLevel() const;
  uint32_t getFaultCode() const;
  AlgorithmHeartbeat generateHeartbeat() const;
  bool shouldTriggerFailsafe() const;

private:
  ModuleHealth current_health_;
  rclcpp::Time last_heartbeat_time_;
  double heartbeat_timeout_;
  bool failsafe_triggered_;
  
  void checkTimeouts();
  uint8_t computeOverallStatus() const;
  uint32_t computeFaultCode() const;
};

} // namespace algorithm_platform
