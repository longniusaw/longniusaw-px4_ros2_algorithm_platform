#!/usr/bin/env python3

# ============================================================
# Communication Node
#
# 支持协议动作：
#   takeoff / hover / fly_to / track_vel / rtl / land / orbit / turn / stop
#
# 全部通过 TrajectorySetpoint 实现：
#   - 位置控制：position
#   - 速度控制：velocity
#   - 偏航控制：yaw
#
# 设计要点：
#   - 抢占式命令：新命令立即替换当前命令
#   - 坐标：协议默认 geo（WGS84），内部转 NED
#   - OffboardControlMode 根据控制模式切换 position / velocity
#   - orbit 分两阶段：先飞到圆周起点，再沿圆周盘旋
#   - land 完成后发送上锁
#   - 状态机带超时保护
# ============================================================

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleAttitude,
)


# ============================================================
# 常量
# ============================================================

EARTH_RADIUS = 6378137.0
DEG_TO_RAD = math.pi / 180.0


class CommunicationNode(Node):

    def __init__(self):
        super().__init__('algws_communication')

        # ---------------- QoS ----------------
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- 订阅命令 ----------------
        self.create_subscription(
            String,
            '/algorithm/command/arbitrated',
            self.command_callback,
            10,
        )

        # ---------------- PX4 发布 ----------------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )

        # ---------------- PX4 订阅 ----------------
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.global_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos,
        )

        # ---------------- 状态变量 ----------------
        self.armed = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        # 本地位置
        self.current = [0.0, 0.0, 0.0]           # NED
        self.target = [0.0, 0.0, 0.0]            # NED
        self.local_pos_valid = False

        # 速度控制
        self.target_vel = [0.0, 0.0, 0.0]        # NED m/s
        self.use_velocity = False

        # 偏航控制
        self.target_yaw = float('nan')
        self.current_yaw = 0.0

        # 地理坐标 / home
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.home_set = False
        self.global_valid = False

        # 当前命令
        self.current_command = None

        # 状态机
        self.state = 'IDLE'
        self.state_entry_time = time.time()

        # Offboard 准备
        self.offboard_counter = 0
        self.offboard_required_count = 40

        # 容差
        self.position_tolerance = 0.3
        self.yaw_tolerance = 0.1

        # 超时（秒）
        self.timeout_prepare = 8.0
        self.timeout_offboard = 5.0
        self.timeout_arm = 5.0

        # orbit 状态
        self.orbit_center_ned = [0.0, 0.0, 0.0]
        self.orbit_radius = 0.0
        self.orbit_speed = 0.0
        self.orbit_angle = 0.0
        self.orbit_last_time = 0.0
        self.orbit_phase = 'idle'                # idle / approach / circle

        # 可调参数
        self.rtl_altitude = 10.0                 # RTL 目标高度（米，正值）

        # ---------------- 定时器 ----------------
        self.create_timer(0.05, self.publish_offboard_mode)   # 20Hz
        self.create_timer(0.02, self.publish_setpoint)        # 50Hz
        self.create_timer(0.05, self.state_machine)           # 20Hz

        self.get_logger().info('🚀 Communication Node 已启动')
        self.get_logger().info(
            '🎯 支持：takeoff / hover / fly_to / track_vel / '
            'rtl / land / orbit / turn / stop'
        )

    # ============================================================
    # 工具函数
    # ============================================================

    def _to_float(self, value, default=0.0):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _set_state(self, state):
        self.state = state
        self.state_entry_time = time.time()

    def now(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    # ============================================================
    # 坐标转换
    # ============================================================

    def geo_to_ned(self, lon, lat, alt):
        """WGS84 经纬高 -> 本地 NED（相对 home）"""
        if not self.home_set:
            self.get_logger().warn('Home 未设置，无法 geo -> NED')
            return [0.0, 0.0, 0.0]

        lat0 = self.home_lat * DEG_TO_RAD
        lon0 = self.home_lon * DEG_TO_RAD
        latr = lat * DEG_TO_RAD
        lonr = lon * DEG_TO_RAD

        x = (latr - lat0) * EARTH_RADIUS
        y = (lonr - lon0) * EARTH_RADIUS * math.cos(lat0)
        z = -(alt - self.home_alt)

        return [x, y, z]

    # ============================================================
    # 回调
    # ============================================================

    def command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'❌ JSON 解析失败: {exc}')
            return

        if not isinstance(data, dict):
            self.get_logger().error('❌ 命令必须是 JSON 对象')
            return

        action = data.get('action')
        if not isinstance(action, str):
            self.get_logger().error('❌ 缺少或非法 action')
            return

        params = data.get('params', {})
        if not isinstance(params, dict):
            self.get_logger().error('❌ params 必须是 JSON 对象')
            return

        coord = data.get('coord', 'geo')

        supported = {
            'takeoff', 'hover', 'fly_to', 'track_vel',
            'rtl', 'land', 'orbit', 'turn', 'stop',
        }
        if action not in supported:
            self.get_logger().warn(f'⚠️ 未实现动作: {action}')
            return

        # 抢占
        if self.current_command is not None:
            old_id = self.current_command.get('action_id', 'unknown')
            self.get_logger().info(
                f'🔄 命令被抢占: {old_id} -> {data.get("action_id")}'
            )

        self.current_command = data
        self.get_logger().info(f'📥 收到命令: {data}')

        self._prepare_action(action, params, coord)

    def status_callback(self, msg):
        self.armed = (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        self.nav_state = msg.nav_state

    def position_callback(self, msg):
        self.current = [float(msg.x), float(msg.y), float(msg.z)]
        # 位置有效性
        self.local_pos_valid = bool(msg.xy_valid and msg.z_valid)

    def global_position_callback(self, msg):
        if msg.lat_lon_valid and msg.alt_valid:
            self.global_valid = True
            if not self.home_set:
                self.home_lat = msg.lat
                self.home_lon = msg.lon
                self.home_alt = msg.alt
                self.home_set = True
                self.get_logger().info(
                    f'🏠 Home 已设置: lat={msg.lat:.7f}, '
                    f'lon={msg.lon:.7f}, alt={msg.alt:.2f}'
                )

    def attitude_callback(self, msg):
        # 四元数 (w, x, y, z) -> yaw
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ============================================================
    # 动作准备
    # ============================================================

    def _prepare_action(self, action, params, coord):
        """
        根据动作设置目标，并决定状态机转移。
        - 若已在 Offboard 且已解锁，直接进入 EXECUTING
        - 若在等待 Offboard / ARM 的过程中，保留当前进度
        - 否则重置并进入 PREPARE
        """
        # 重置控制模式
        self.use_velocity = False
        self.target_yaw = float('nan')
        self.orbit_phase = 'idle'

        already_offboard = (
            self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.armed
        )
        in_wait_state = self.state in (
            'PREPARE', 'WAIT_OFFBOARD', 'WAIT_ARM'
        )

        # ---------------- stop 特殊处理 ----------------
        if action == 'stop':
            if already_offboard:
                # 原地悬停：目标设为当前位置
                self.target = list(self.current)
                self._set_state('EXECUTING')
                self.get_logger().info('⏹ stop：原地悬停')
            else:
                # 未在 Offboard：直接结束，不做任何飞行动作
                self.get_logger().info('⏹ stop：当前未 Offboard，直接结束')
                self.finish_current_command()
            return

        # ---------------- takeoff ----------------
        if action == 'takeoff':
            if not self.local_pos_valid:
                self.get_logger().warn(
                    '⚠️ takeoff：本地位置无效，仍以当前值作为水平基准'
                )
            alt = abs(self._to_float(params.get('alt'), 2.0))
            self.target = [
                self.current[0],
                self.current[1],
                -alt,
            ]

        # ---------------- hover ----------------
        elif action == 'hover':
            pos = params.get('pos')
            if isinstance(pos, list) and len(pos) >= 3:
                lon = self._to_float(pos[0])
                lat = self._to_float(pos[1])
                alt = self._to_float(pos[2])
                if coord == 'geo':
                    self.target = self.geo_to_ned(lon, lat, alt)
                else:
                    self.target = [lon, lat, alt]
            else:
                self.target = list(self.current)

        # ---------------- fly_to ----------------
        elif action == 'fly_to':
            pos = params.get('pos')
            if isinstance(pos, list) and len(pos) >= 3:
                lon = self._to_float(pos[0])
                lat = self._to_float(pos[1])
                alt = self._to_float(pos[2])
                if coord == 'geo':
                    self.target = self.geo_to_ned(lon, lat, alt)
                else:
                    self.target = [lon, lat, alt]
            else:
                # 缺省：当前上方 5m
                self.target = [
                    self.current[0],
                    self.current[1],
                    self.current[2] - 5.0,
                ]

        # ---------------- track_vel ----------------
        elif action == 'track_vel':
            vel = params.get('vel')
            if isinstance(vel, list) and len(vel) >= 3:
                self.target_vel = [
                    self._to_float(vel[0]),
                    self._to_float(vel[1]),
                    self._to_float(vel[2]),
                ]
            else:
                self.target_vel = [0.0, 0.0, 0.0]
            self.use_velocity = True

        # ---------------- rtl ----------------
        elif action == 'rtl':
            if self.home_set:
                self.target = [
                    0.0,
                    0.0,
                    -abs(self.rtl_altitude),
                ]
            else:
                self.get_logger().warn('⚠️ RTL：Home 未设置，保持当前位置')
                self.target = list(self.current)

        # ---------------- land ----------------
        elif action == 'land':
            self.target = [self.current[0], self.current[1], 0.0]

        # ---------------- orbit ----------------
        elif action == 'orbit':
            center = params.get('center')
            self.orbit_radius = abs(
                self._to_float(params.get('radius'), 10.0)
            )
            self.orbit_speed = abs(
                self._to_float(params.get('speed'), 2.0)
            )

            if isinstance(center, list) and len(center) >= 3:
                if coord == 'geo':
                    self.orbit_center_ned = self.geo_to_ned(
                        self._to_float(center[0]),
                        self._to_float(center[1]),
                        self._to_float(center[2]),
                    )
                else:
                    self.orbit_center_ned = [
                        self._to_float(center[0]),
                        self._to_float(center[1]),
                        self._to_float(center[2]),
                    ]
            else:
                self.orbit_center_ned = list(self.current)

            # 圆周起点：角度 0
            start_x = self.orbit_center_ned[0] + self.orbit_radius
            start_y = self.orbit_center_ned[1]
            start_z = self.orbit_center_ned[2]
            self.target = [start_x, start_y, start_z]
            self.orbit_angle = 0.0
            self.orbit_phase = 'approach'

        # ---------------- turn ----------------
        elif action == 'turn':
            self.target_yaw = self._to_float(params.get('yaw'), 0.0)
            # 保持当前位置，仅控制偏航
            self.target = list(self.current)

        # ---------------- 状态转移 ----------------
        if already_offboard:
            self._set_state('EXECUTING')
        elif in_wait_state:
            # 保留当前 Offboard 准备进度
            self.get_logger().info('⏳ 保持当前 Offboard 准备进度')
        else:
            self.offboard_counter = 0
            self._set_state('PREPARE')

    # ============================================================
    # 状态机
    # ============================================================

    def state_machine(self):
        now = time.time()

        # ---------------- IDLE ----------------
        if self.state == 'IDLE':
            if self.current_command is not None:
                self._set_state('PREPARE')

        # ---------------- PREPARE ----------------
        elif self.state == 'PREPARE':
            if now - self.state_entry_time > self.timeout_prepare:
                self.get_logger().error('❌ PREPARE 超时')
                self.finish_current_command()
                return

            if self.offboard_counter >= self.offboard_required_count:
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )
                self.get_logger().info('🔄 请求 OFFBOARD')
                self._set_state('WAIT_OFFBOARD')

        # ---------------- WAIT_OFFBOARD ----------------
        elif self.state == 'WAIT_OFFBOARD':
            if now - self.state_entry_time > self.timeout_offboard:
                self.get_logger().error('❌ WAIT_OFFBOARD 超时')
                self.finish_current_command()
                return

            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info('✅ OFFBOARD 已激活')
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                )
                self._set_state('WAIT_ARM')

        # ---------------- WAIT_ARM ----------------
        elif self.state == 'WAIT_ARM':
            if now - self.state_entry_time > self.timeout_arm:
                self.get_logger().error('❌ WAIT_ARM 超时')
                self.finish_current_command()
                return

            if self.armed:
                self.get_logger().info('✅ 已解锁')
                self._set_state('EXECUTING')

        # ---------------- EXECUTING ----------------
        elif self.state == 'EXECUTING':
            if self.current_command is None:
                self._set_state('IDLE')
                return
            self._execute_current(now)

    # ============================================================
    # 执行当前命令
    # ============================================================

    def _execute_current(self, now):
        action = self.current_command.get('action')

        # ---------------- track_vel：持续执行 ----------------
        if action == 'track_vel':
            return

        # ---------------- turn ----------------
        if action == 'turn':
            if abs(self.current_yaw - self.target_yaw) < self.yaw_tolerance:
                self.get_logger().info('✅ turn 完成')
                self.finish_current_command()
            return

        # ---------------- orbit ----------------
        if action == 'orbit':
            if self.orbit_phase == 'approach':
                if self.distance_to_target() <= self.position_tolerance:
                    self.orbit_phase = 'circle'
                    self.orbit_last_time = now
                    self.get_logger().info('🔄 orbit：进入盘旋')
                return

            if self.orbit_phase == 'circle':
                dt = now - self.orbit_last_time
                self.orbit_last_time = now
                omega = self.orbit_speed / max(self.orbit_radius, 0.1)
                self.orbit_angle += omega * dt

                x = self.orbit_center_ned[0] + \
                    self.orbit_radius * math.cos(self.orbit_angle)
                y = self.orbit_center_ned[1] + \
                    self.orbit_radius * math.sin(self.orbit_angle)
                z = self.orbit_center_ned[2]

                self.target = [x, y, z]
            return

        # ---------------- land ----------------
        if action == 'land':
            if self.distance_to_target() <= self.position_tolerance:
                self.get_logger().info('✅ land 完成，发送上锁')
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    0.0,
                )
                self.finish_current_command()
            return

        # ---------------- 位置类动作 ----------------
        if action in ('takeoff', 'hover', 'fly_to', 'rtl', 'stop'):
            if self.distance_to_target() <= self.position_tolerance:
                self.get_logger().info(f'✅ {action} 完成')
                self.finish_current_command()
            return

    # ============================================================
    # 发布 OffboardControlMode
    # ============================================================

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now()

        # 根据当前控制模式，只能使能一种
        if self.use_velocity:
            msg.position = False
            msg.velocity = True
        else:
            msg.position = True
            msg.velocity = False

        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)
        self.offboard_counter += 1

    # ============================================================
    # 发布 TrajectorySetpoint
    # ============================================================

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now()

        if self.use_velocity:
            msg.position = [float('nan')] * 3
            msg.velocity = [float(v) for v in self.target_vel]
        else:
            msg.position = [float(v) for v in self.target]
            msg.velocity = [float('nan')] * 3

        msg.acceleration = [float('nan')] * 3

        if math.isnan(self.target_yaw):
            msg.yaw = float('nan')
        else:
            msg.yaw = float(self.target_yaw)

        msg.yawspeed = float('nan')

        self.setpoint_pub.publish(msg)

    # ============================================================
    # VehicleCommand
    # ============================================================

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.command_pub.publish(msg)
        self.get_logger().info(f'📤 VehicleCommand: {command}')

    # ============================================================
    # 工具
    # ============================================================

    def distance_to_target(self):
        dx = self.current[0] - self.target[0]
        dy = self.current[1] - self.target[1]
        dz = self.current[2] - self.target[2]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def finish_current_command(self):
        if self.current_command is not None:
            action_id = self.current_command.get('action_id', 'unknown')
            action = self.current_command.get('action', 'unknown')
            self.get_logger().info(
                f'🏁 命令完成 id={action_id}, action={action}'
            )

        self.current_command = None
        self.use_velocity = False
        self.target_yaw = float('nan')
        self.orbit_phase = 'idle'
        self._set_state('IDLE')


# ============================================================
# main
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()