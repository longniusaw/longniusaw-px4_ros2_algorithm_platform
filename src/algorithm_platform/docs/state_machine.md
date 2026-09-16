# 故障状态机跳转表

## 状态定义
- NORMAL: 正常运行
- DEGRADED: 降级运行
- FAILSAFE: 保护模式
- LANDING: 降落中

## 跳转条件
| 当前状态 | 触发条件 | 目标状态 | 动作 |
|---------|---------|---------|------|
| NORMAL | SLAM丢失 | DEGRADED | 减速悬停 |
| DEGRADED | 心跳超时 | FAILSAFE | 切Offboard |
| FAILSAFE | 状态恢复 | NORMAL | 恢复任务 |
