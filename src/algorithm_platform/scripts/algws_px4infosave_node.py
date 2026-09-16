#!/usr/bin/env python3
# scripts/algws_px4infosave_node.py
# 接收 PX4 状态信息（通过 uXRCE-DDS），缓存并发布整合状态

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 消息类型
try:
    from px4_msgs.msg import VehicleGlobalPosition, VehicleAttitude, VehicleStatus
except ImportError:
    raise ImportError(
        "px4_msgs not found. Please install px4_ros_com or ensure px4_msgs is in your workspace."
    )

from std_msgs.msg import String
import json
import time
import threading
from typing import Optional, Dict, Any

class PX4InfoSaveNode(Node):
    def __init__(self):
        super().__init__('algws_px4infosave')

        # ---------- 配置参数 ----------
        self.declare_parameter('publish_rate', 1.0)          # 整合消息发布频率 (Hz)
        self.declare_parameter('save_to_file', False)        # 是否保存到本地文件
        self.declare_parameter('file_path', './px4_log.csv') # 文件路径

        publish_rate = self.get_parameter('publish_rate').value
        self.save_to_file = self.get_parameter('save_to_file').value
        self.file_path = self.get_parameter('file_path').value

        # ---------- QoS 设置 ----------
        # PX4 话题通常使用可靠的、最佳努力传输，此处使用与 px4_ros_com 一致的 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------- 订阅 PX4 话题 ----------
        self.global_pos_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.global_position_callback,
            qos
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            qos
        )

        # ---------- 发布整合后的状态（JSON 字符串） ----------
        self.status_pub = self.create_publisher(
            String,
            '/px4/status',          # 自定义话题，供其他节点订阅
            qos
        )

        # ---------- 缓存数据（线程安全） ----------
        self.lock = threading.Lock()
        self.latest_global_pos: Optional[VehicleGlobalPosition] = None
        self.latest_attitude: Optional[VehicleAttitude] = None
        self.latest_vehicle_status: Optional[VehicleStatus] = None

        # 时间戳（用于记录）
        self.last_update_time = time.time()

        # 文件记录（如果启用）
        if self.save_to_file:
            self._init_csv_file()

        # 定时发布整合状态
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_integrated_status)

        self.get_logger().info('🚀 PX4 Info Save Node 已启动')
        self.get_logger().info(f'📡 发布频率: {publish_rate} Hz')
        self.get_logger().info(f'📁 保存到文件: {self.save_to_file}')

    # ---------- PX4 话题回调 ----------
    def global_position_callback(self, msg: VehicleGlobalPosition):
        with self.lock:
            self.latest_global_pos = msg
            self.last_update_time = time.time()
        self.get_logger().debug('📍 收到 GlobalPosition 更新')

    def attitude_callback(self, msg: VehicleAttitude):
        with self.lock:
            self.latest_attitude = msg
        self.get_logger().debug('🔄 收到 Attitude 更新')

    def vehicle_status_callback(self, msg: VehicleStatus):
        with self.lock:
            self.latest_vehicle_status = msg
        self.get_logger().debug('📊 收到 VehicleStatus 更新')

    # ---------- 整合与发布 ----------
    def publish_integrated_status(self):
        """构建整合的 JSON 状态并发布，同时可能写入文件"""
        with self.lock:
            if not self.latest_global_pos:
                # 尚未收到任何数据，不发布
                return

            # 构建完整的状态字典
            status_dict = self._build_status_dict()

        # 发布 JSON 字符串
        msg = String()
        msg.data = json.dumps(status_dict, indent=None, separators=(',', ':'))
        self.status_pub.publish(msg)
        self.get_logger().debug('📤 发布整合状态')

        # 可选写入 CSV
        if self.save_to_file:
            self._write_to_csv(status_dict)

    def _build_status_dict(self) -> Dict[str, Any]:
        """从缓存数据中构建统一的状态字典"""
        pos = self.latest_global_pos
        att = self.latest_attitude
        stat = self.latest_vehicle_status

        # 基础信息
        data = {
            "timestamp": int(time.time() * 1e6),  # 微秒，与 PX4 一致
            "global_position": {
                "lat": pos.lat if pos else 0.0,
                "lon": pos.lon if pos else 0.0,
                "alt": pos.alt if pos else 0.0,
                "alt_ellipsoid": pos.alt_ellipsoid if pos else 0.0,
                "eph": pos.eph if pos else 0.0,
                "epv": pos.epv if pos else 0.0,
                "terrain_alt": pos.terrain_alt if pos else 0.0,
                "dead_reckoning": pos.dead_reckoning if pos else False,
                "lat_lon_valid": pos.lat_lon_valid if pos else False,
                "alt_valid": pos.alt_valid if pos else False,
            },
            "attitude": {
                "q_w": att.q[0] if att else 0.0,
                "q_x": att.q[1] if att else 0.0,
                "q_y": att.q[2] if att else 0.0,
                "q_z": att.q[3] if att else 0.0,
                "quat_reset_counter": att.quat_reset_counter if att else 0,
            },
            "vehicle_status": {
                "arming_state": stat.arming_state if stat else 0,
                "nav_state": stat.nav_state if stat else 0,
                "failsafe": stat.failsafe if stat else False,
                "gcs_connection_lost": stat.gcs_connection_lost if stat else False,
                "pre_flight_checks_pass": stat.pre_flight_checks_pass if stat else False,
                "vehicle_type": stat.vehicle_type if stat else 0,
                "system_type": stat.system_type if stat else 0,
            }
        }

        # 如果有 home 信息（可从 vehicle_status 或其他来源获取，这里先留空，可后续扩展）
        # 协议中 home 通常由 commandin 节点从 PX4 获取，这里可以不重复提供

        return data

    # ---------- CSV 保存 ----------
    def _init_csv_file(self):
        """创建 CSV 文件并写入表头（如果文件不存在）"""
        import os
        import csv
        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # 写表头（根据 _build_status_dict 的键）
                # 为简化，只写入主要字段
                header = [
                    'timestamp', 
                    'lat', 'lon', 'alt',
                    'q_w', 'q_x', 'q_y', 'q_z',
                    'arming_state', 'nav_state', 'failsafe'
                ]
                writer.writerow(header)
            self.get_logger().info(f'📁 CSV 文件已创建: {self.file_path}')

    def _write_to_csv(self, data: Dict):
        """将当前状态写入 CSV 文件（追加模式）"""
        import csv
        try:
            with open(self.file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                row = [
                    data['timestamp'],
                    data['global_position']['lat'],
                    data['global_position']['lon'],
                    data['global_position']['alt'],
                    data['attitude']['q_w'],
                    data['attitude']['q_x'],
                    data['attitude']['q_y'],
                    data['attitude']['q_z'],
                    data['vehicle_status']['arming_state'],
                    data['vehicle_status']['nav_state'],
                    data['vehicle_status']['failsafe'],
                ]
                writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f'❌ 写入 CSV 失败: {e}')

# ---------- 主函数 ----------
def main(args=None):
    rclpy.init(args=args)
    node = PX4InfoSaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()