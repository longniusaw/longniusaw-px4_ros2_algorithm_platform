# 模块接口契约文档

## 1. 发布话题
### /offboard/trajectory_setpoint
- 类型: TrajectorySetpoint
- 频率: 30-50Hz
- 说明: MPU 输出的期望位置/速度

### /offboard/algorithm_heartbeat
- 类型: AlgorithmHeartbeat
- 频率: 5-10Hz
- 说明: 算法健康心跳

## 2. 订阅话题
### /px4/vehicle_status
- 类型: px4_msgs/VehicleStatus
- 说明: PX4 飞行状态回传
