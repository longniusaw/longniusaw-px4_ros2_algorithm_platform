#!/usr/bin/env python3
"""
mock_gcs —— 假地面站: 给 PX4 发 HEARTBEAT, 让飞控认为 GCS 在线
===============================================================
解决场景: 纯 ROS2/无 QGC 跑 SITL 时, PX4(新版 NAV_DLL_ACT 默认启用)预检
"No connection to the GCS" 会拒绝解锁, 导致无人机没反应。

原理: PX4 的 MAVLink 模块把"收到外部 HEARTBEAT"视为地面站在线,
本脚本向 PX4 SITL 的 GCS 端口(本地 UDP)回发 GCS 心跳即可。

PX4 SITL 多机: 每台 instance 的 GCS 端口 = 14550 + instance 号,
   px4 -i 0 -> 14550,  px4 -i 1 -> 14551,  px4 -i 2 -> 14552 ...

用法:
  python3 scripts/mock_gcs.py                 # instance 0, UDP 14550
  python3 scripts/mock_gcs.py --port 14551    # instance 1
  ros2 run bmf_onboard mock_gcs --port 14551  # (构建安装后)
"""
import argparse
import time

from pymavlink import mavutil

DEFAULT_PORT = 14550   # PX4 SITL instance 0 的 GCS 端口(本地监听)


def main() -> None:
    parser = argparse.ArgumentParser(description="假地面站: 给 PX4 SITL 回发 GCS 心跳")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="本机监听的 UDP 端口(instance i -> 14550+i)")
    args = parser.parse_args()
    udp_port = args.port

    # udpin = 作为地面站监听; source 255/190 是标准 GCS 身份
    conn = mavutil.mavlink_connection(
        f"udpin:127.0.0.1:{udp_port}", source_system=255, source_component=190)
    print(f"[mock_gcs] 监听 127.0.0.1:{udp_port}, 等待 PX4 heartbeat...")
    conn.wait_heartbeat(timeout=10)
    print(f"[mock_gcs] 收到 PX4 heartbeat (sysid={conn.target_system}), 开始回发 GCS heartbeat")
    while True:
        # MAV_TYPE_GCS + 全零状态 = 标准地面站心跳
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                0, 0, 0)
        time.sleep(1)


if __name__ == "__main__":
    main()
