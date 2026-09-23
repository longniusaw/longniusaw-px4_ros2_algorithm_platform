algws

git add .

git commit -m "描述你改了什么"

git push

# 下发动作协议（/action_intent ，D → C → A）

动作原语经 `/action_intent` 下发，立即执行、可随时被新动作/任务替换或打断。

## 报文结构

```json
{
  "action_id": "act_<时间戳>",   // 可选，缺省 A 端自动生成；网页自动生成 act_ 前缀
  "action":  "<动作名>",          // 必填，白名单见下
  "params":  { ... },             // 各动作参数，见下
  // "coord": "geo"               // 可选，缺省 geo（经纬高 WGS84）
}
```

- `coord` 缺省即 `geo`（WGS84 经纬高）；坐标为 **[经度, 纬度, 高度]** 顺序。
- `coord: "ned"` 兼容（坐标为 [x, y, z] 米，NED），一般外部不用。

## 动作白名单

`takeoff / hover / fly_to / track_vel / rtl / land / orbit / turn / stop`

## 各动作参数

| 动作 | params 字段 | 说明 |
|---|---|---|
| takeoff | `alt` | 起飞高度，相对米（正值） |
| hover | `pos`（可选）| 悬停点 [经度,纬度,高度]；缺省=当前位置 |
| fly_to | `pos` | 目标点 [经度,纬度,高度]；缺省=当前上方默认高 |
| track_vel | `vel` | NED 速度 [北,东,下](m/s)，持续朝该方向飞 |
| orbit | `center` `radius` `speed` | 圆心 [经度,纬度,高度] + 半径(m) + 速度(m/s) |
| turn | `yaw` | 偏航角（弧度） |
| rtl | — | 返航，无参数 |
| land | — | 降落，无参数 |
| stop | — | 停止（取消当前任务/动作，原地悬停），无参数 |

## 完整示例

### 1. 起飞 takeoff

```json
{"action_id":"act_1788227300713","action":"takeoff","params":{"alt":20}}
```

### 2. 悬停 hover（原位）

```json
{"action_id":"act_1788227300714","action":"hover","params":{}}
```

或指定悬停点：

```json
{"action_id":"act_1788227300715","action":"hover","params":{"pos":[114.305823,30.593250,20]}}
```

### 3. 飞往 fly_to

```json
{"action_id":"act_1788227300716","action":"fly_to","params":{"pos":[114.305823,30.593250,20]}}
```

### 4. 速度控制 track_vel

```json
{"action_id":"act_1788227300717","action":"track_vel","params":{"vel":[2,0,0]}}
```

### 5. 盘旋 orbit

```json
{"action_id":"act_1788227300718","action":"orbit","params":{"center":[114.305823,30.593250,20],"radius":15,"speed":3}}
```

### 6. 转向 turn

```json
{"action_id":"act_1788227300719","action":"turn","params":{"yaw":1.5708}}
```

### 7. 返航 rtl

```json
{"action_id":"act_1788227300720","action":"rtl","params":{}}
```

### 8. 降落 land

```json
{"action_id":"act_1788227300721","action":"land","params":{}}
```

### 9. 停止 stop

```json
{"action_id":"act_1788227300722","action":"stop","params":{}}
```

## 日志对照

网页下发后日志形如：

```
下发动作 → uav_1/action_intent {"action_id":"act_1788227300713","action":"takeoff","params":{"alt":20}}
```

---

# 下发航点任务协议（/mission_intent ，D → C → A）

## 报文结构

```json
{
  "mission_id": "mission_<时间戳>",   // 必填
  "mission_type": 1,                  // 必填; 1=航点
  "title": "航点任务",                // 可选, 任务标题(仅展示)
  "waypoints": [                      // 必填, 每点 [经度, 纬度, 高度]
    [114.305299, 30.592800, 20],
    [114.305823, 30.593250, 20]
  ]
}
```

- `coord` 缺省即 `geo`（WGS84 经纬高），坐标为 **[经度, 纬度, 高度]**。
- A 端 `decode_mission` 自动补齐缺省字段：`target_classes`、`altitude_m`、`speed_mps`、`center`、`radius_m`、`max_duration_s` 等（航点任务一般无需填）。

## 完整示例

```json
{"mission_id":"mission_1788227450000","mission_type":1,"title":"航点任务","waypoints":[[114.305299,30.592800,20],[114.305823,30.593250,20],[114.306449,30.593789,20],[114.307075,30.594328,20]]}
```

## 日志对照

网页下发后日志形如：

```
下发任务 → uav_1/mission_intent {"mission_id":"mission_1788227450000","mission_type":1,"title":"航点任务","waypoints":[[114.305299,30.592800,20],[114.305823,30.593250,20],[114.306449,30.593789,20],[114.307075,30.594328,20]]}
```

---

# 计划提案（/plan_proposal，A → C → D）

A 端在需要审批时（`require_plan_ack: true`）发出，坐标为 **[经度, 纬度, 高度]**，`require_plan_ack: true` 时自主决策。

## 完整示例（初始航点任务）

```json
{"mission_id":"mission_1788227450000","plan_id":"plan_1788227451000","takeoff_alt":20.0,"hover_pos":[114.3053,30.5928,30.0],"cruise_speed_mps":2.0,"waypoints":[[114.305299,30.592800,30.0],[114.305823,30.593250,30.0],[114.306449,30.593789,30.0],[114.307075,30.594328,30.0]]}
```

## 完整示例（重规划后的提案）

```json
{"mission_id":"mission_1788227450000","plan_id":"plan_detour_1788227452000","takeoff_alt":20.0,"hover_pos":[114.306459,30.593672,30.0],"cruise_speed_mps":2.0,"waypoints":[[114.306459,30.593672,30.0],[114.306448,30.593789,30.0],[114.307074,30.594328,30.0]]}
```

## 日志对照

```
收到计划提案 ← uav_1/plan_proposal: {"mission_id":"mission_1788227450000","plan_id":"plan_1788227451000","takeoff_alt":20.0,"hover_pos":[114.3053,30.5928,30.0],"cruise_speed_mps":2.0,"waypoints":[...]}
```

---

# 计划确认（/plan_ack，D → C → A）

对 `/plan_proposal` 的提案作出批准/驳回。`plan_id` 必须与提案一致。

## 报文结构

```json
{
  "mission_id": "mission_...",  // 任务 id（与提案一致）
  "plan_id":    "plan_...",      // 提案/绕行的 plan_id（与提案一致）
  "approved":   true             // true=批准, false=驳回
}
```

## 完整示例

批准：

```json
{"mission_id":"mission_1788227450000","plan_id":"plan_1788227451000","approved":true}
```

驳回：

```json
{"mission_id":"mission_1788227450000","plan_id":"plan_1788227451000","approved":false}
```

## 日志对照

```
批准 → uav_1/plan_ack {"mission_id":"mission_1788227450000","plan_id":"plan_1788227451000","approved":true}
驳回 → uav_1/plan_ack {"mission_id":"mission_1788227450000","plan_id":"plan_1788227451000","approved":false}
```

---

# 状态上报（/mission_status，A → C → D，1Hz）

A 端 1Hz 上报一次当前状态（无任务也发）。JSON 字符串。

## 顶层字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `mission_id` | string | 当前任务 id（无任务为空串） |
| `state` | number | 状态机数值契约（HOVER=1/TRACK=2/RTL=3/COMPLETE=4） |
| `state_str` | string | 可读状态名（`HOVER`/`WAYPOINT`/`WAIT_ACK`…） |
| `progress` | number | 任务进度 0~1（航点=已到点数/总数） |
| `pos_ned` | [x,y,z] | 当前 NED 位置（内部米制，无则 [0,0,0]） |
| `battery_pct` | number | 电量%（当前恒 0，未订阅） |
| `target_visible` | bool | 感知是否已确认发现目标 |
| `last_target_class` | string | 最近目标类别 |
| `last_target_conf` | number | 最近目标置信度 |
| `plan_id` | string | 当前计划 id（重规划后变化） |
| `alarm` | string | 保命/告警（当前空） |
| `debug` | object | 调试信息（见下） |
| `history` | array | 任务/规划/动作历史（见下） |

## `debug` 对象

| 字段 | 说明 |
|---|---|
| `px4_online` | PX4 是否在线 |
| `pos_age_s` | 位置龄（秒） |
| `last_alarm` | 最近保命/错误信息 |
| `action` | 当前动作原语 cmd（如 `fly_to`） |
| `det` | 检测摘要 `{count, detections:[{class,conf,w,h}]}` |
| `topics` | 关键话题 hz 统计（红绿灯） |
| `use_b_control` / `require_plan_ack` | 开关配置 |
| `topic_prefix` | 话题前缀（如 `uav_1`），前端用于一致性校验 |
| `setpoint_z` | 当前目标高度(NED z) |
| `vel` | 速度 NED `[vx,vy,vz]` |
| `att` | 姿态 `{roll, pitch, yaw}`（度） |
| `home_lat/lon/alt` / `home_src` | 经纬高基准 + 来源(`px4/gps/none`) |
| `arm_state` `nav_state` `preflight_ok` `bad_flags` | PX4 诊断 |
| `gps_fix` `gps_sats` `gps_lat/lon/alt` | GPS 状态 / 当前经纬高 |
| `last_cmd` | 最近指令应答 |
| `detour_pending` / `detour_plan_id` | 绕行重规划是否在等 D 确认及 plan_id |

## `history` 数组

- **任务**：`{kind:"mission", mission_id, active, status, created_at, finished_at, title, payload, plans:[{plan_id, active, states, created_at, finished_at, waypoints?, decision?}]}`
- **动作**：`{kind:"action", action_id, action, title, detail, payload, active, status, created_at, finished_at}`

## 完整示例（节选）

```json
{
  "mission_id": "",
  "state": 0,
  "state_str": "IDLE",
  "progress": 0.0,
  "pos_ned": [
    0.05765802785754204,
    0.006960552651435137,
    -0.02950507402420044
  ],
  "battery_pct": 0.0,
  "target_visible": false,
  "last_target_class": "",
  "last_target_conf": 0.0,
  "plan_id": "",
  "alarm": "",
  "debug": {
    "px4_online": true,
    "pos_age_s": 0.0,
    "last_alarm": "",
    "action": "",
    "det": null,
    "topics": [],
    "use_b_control": false,
    "require_plan_ack": true,
    "topic_prefix": "uav_1",
    "setpoint_z": null,
    "vel": [
      -0.0,
      0.01,
      -0.0
    ],
    "att": {
      "roll": 0.3,
      "pitch": 0.2,
      "yaw": 72.5
    },
    "home_lat": 30.592800461791022,
    "home_lon": 114.30529992273846,
    "home_alt": 0.24537408351898193,
    "home_src": "px4",
    "arm_state": 1,
    "nav_state": 14,
    "preflight_ok": false,
    "bad_flags": [
      "manual_control_signal_lost",
      "offboard_control_signal_lost",
      "auto_mission_missing",
      "remote_id_unhealthy"
    ],
    "gps_fix": 3,
    "gps_sats": 10,
    "gps_lat": 30.592800600505495,
    "gps_lon": 114.30530009703894,
    "gps_alt": 0.23244939744472504,
    "last_cmd": {
      "cmd": 176,
      "result": 0
    },
    "detour_pending": false,
    "detour_plan_id": ""
  },
  "history": [
    {
      "kind": "action",
      "action_id": "act_1788227300713",
      "action": "rtl",
      "title": "返航",
      "detail": "",
      "payload": {
        "coord": "geo"
      },
      "active": false,
      "status": "成功",
      "created_at": 1788227395,
      "finished_at": 1788227477,
      "states": [
        "WAYPOINT"
      ]
    },
    {
      "kind": "mission",
      "mission_id": "mission_1788227300713",
      "active": false,
      "last_alarm": "",
      "status": "被替换",
      "created_at": 1788227338,
      "title": "航点任务",
      "payload": {
        "title": "航点任务",
        "waypoints": [
          [
            114.305299,
            30.5928,
            20.0
          ],
          [
            114.305823,
            30.59325,
            20.0
          ],
          [
            114.306449,
            30.593789,
            20.0
          ],
          [
            114.307075,
            30.594328,
            20.0
          ]
        ],
        "coord": "geo",
        "target_classes": [],
        "altitude_m": 0.0,
        "speed_mps": 0.0,
        "center": [
          0.0,
          0.0,
          0.0
        ],
        "max_duration_s": 0.0
      },
      "plans": [
        {
          "plan_id": "plan_1788227337",
          "active": false,
          "states": [
            "WAIT_ACK",
            "TAKEOFF",
            "WAYPOINT"
          ],
          "created_at": 1788227338,
          "decision": "approved",
          "finished_at": 1788227360
        },
        {
          "plan_id": "plan_detour_1788227360",
          "active": false,
          "states": [
            "WAYPOINT"
          ],
          "created_at": 1788227360,
          "waypoints": [
            [
              114.3058251074082,
              30.59312561731761,
              19.940812468528748
            ],
            [
              114.30582313439491,
              30.59325055768537,
              20.00494158267975
            ],
            [
              114.3064491343985,
              30.59378955768537,
              20.00494158267975
            ],
            [
              114.30707513440211,
              30.59432855768537,
              20.00494158267975
            ]
          ],
          "decision": "approved",
          "finished_at": 1788227395
        }
      ],
      "finished_at": 1788227395
    },
    {
      "kind": "action",
      "action_id": "act_1788227300713",
      "action": "land",
      "title": "降落",
      "detail": "",
      "payload": {
        "coord": "geo"
      },
      "active": false,
      "status": "成功",
      "created_at": 1788227315,
      "finished_at": 1788227335,
      "states": [
        "EXEC"
      ]
    },
    {
      "kind": "action",
      "action_id": "act_1788227300713",
      "action": "takeoff",
      "title": "起飞",
      "detail": "高20.0m",
      "payload": {
        "coord": "geo",
        "alt": 20.0
      },
      "active": false,
      "status": "被替换",
      "created_at": 1788227305,
      "finished_at": 1788227315,
      "states": [
        "IDLE"
      ]
    }
  ]
}
```

---

# PX4 订阅字段（A 端直连飞控的话题，A → C）

## 全局位置 `/fmu/out/vehicle_global_position`（VehicleGlobalPosition）

| 字段 | 类型 | 说明 |
|---|---|---|
| `timestamp` | uint64 | 自系统启动时间(微秒) |
| `timestamp_sample` | uint64 | 原始数据采样时间(微秒) |
| `lat` | float64 | 纬度(度) |
| `lon` | float64 | 经度(度) |
| `alt` | float32 | 高度 AMSL(米) |
| `alt_ellipsoid` | float32 | 椭球面以上高度(米) |
| `lat_lon_valid` | bool | 经纬度有效 |
| `alt_valid` | bool | 高度有效 |
| `delta_alt` | float32 | 高度重置增量 |
| `delta_terrain` | float32 | 地形重置增量 |
| `lat_lon_reset_counter` | uint8 | 水平位置重置计数 |
| `alt_reset_counter` | uint8 | 高度重置计数 |
| `terrain_reset_counter` | uint8 | 地形重置计数 |
| `eph` | float32 | 水平位置误差标准差(米) |
| `epv` | float32 | 垂直位置误差标准差(米) |
| `terrain_alt` | float32 | 地形高度 WGS84(米) |
| `terrain_alt_valid` | bool | 地形高度估计有效 |
| `dead_reckoning` | bool | 是否通过航位推算估计 |

## 姿态 `/fmu/out/vehicle_attitude`（VehicleAttitude）

| 字段 | 类型 | 说明 |
|---|---|---|
| `timestamp` | uint64 | 自系统启动时间(微秒) |
| `timestamp_sample` | uint64 | 原始数据采样时间(微秒) |
| `q[4]` | float32[] | 四元数 (w,x,y,z；Hamilton，FRD 机体→NED 地面) |
| `delta_q_reset` | float32[4] | 最近重置期间四元数变化量 |
| `quat_reset_counter` | uint8 | 四元数重置计数 |

## 飞控状态 `/fmu/out/vehicle_status_v4`（VehicleStatus）

| 字段 | 类型 | 说明 |
|---|---|---|
| `timestamp` | uint64 | 自系统启动时间(微秒) |
| `armed_time` | uint64 | 解锁时间(微秒) |
| `takeoff_time` | uint64 | 起飞时间(微秒) |
| `arming_state` | uint8 | 解锁状态(1=未解锁, 2=已解锁) |
| `latest_arming_reason` | uint8 | 最近解锁原因 |
| `latest_disarming_reason` | uint8 | 最近解锁原因 |
| `nav_state_timestamp` | uint64 | 当前导航态激活时间 |
| `nav_state_user_intention` | uint8 | 用户选择的模式 |
| `nav_state` | uint8 | 当前激活模式(可读映射见前端 NAV_STATE 表) |
| `executor_in_charge` | uint8 | 当前模式执行器(0=Autopilot) |
| `nav_state_display` | uint8 | 经 MAVLink 的用户可见导航态 |
| `accepts_offboard_setpoints` | bool | 当前模式是否接受 offboard setpoint |
| `valid_nav_states_mask` | uint32 | 有效导航态掩码 |
| `can_set_nav_states_mask` | uint32 | 可设导航态掩码 |
| `hil_state` | uint8 | HIL 状态 |
| `vehicle_type` | uint8 | 飞机类型(1=旋翼, 2=固定翼, 3=地面车) |
| `failsafe` | bool | 是否处于 failsafe(如 RTL/Hover/Terminate) |
| `failsafe_and_user_took_over` | bool | failsafe 且用户接管 |
| `failsafe_defer_state` | uint8 | failsafe 推迟状态 |
| `gcs_connection_lost` | bool | 与 GCS 断链 |
| `gcs_connection_lost_counter` | uint8 | 断链事件计数 |
| `high_latency_data_link_lost` | bool | 高延迟链路断链 |
| `is_vtol` / `is_vtol_tailsitter` | bool | VTOL 相关标志 |
| `in_transition_mode` / `in_transition_to_fw` | bool | VTOL 过渡标志 |
| `system_type` / `system_id` / `component_id` | uint8 | MAVLink 标识 |
| `safety_button_available` / `safety_off` | bool | 安全开关 |
| `power_input_valid` / `usb_connected` | bool | 供电/USB |
| `open_drone_id_system_present/healthy` | bool | OpenDroneID |
| `parachute_system_present/healthy` | bool | 降落伞系统 |
| `traffic_avoidance_system_present` | bool | 避障系统 |
| `rc_calibration_in_progress` / `calibration_enabled` | bool | 校准标志 |
| `pre_flight_checks_pass` | bool | 所有校核通过(可解锁) |
