#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

class TakeoffCommandNode(Node):
    def __init__(self):
        super().__init__('takeoff_command_node')
        # 创建发布者，话题为 '/fmu/in/vehicle_command'
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        # 在节点启动后 1 秒发布一次命令（给系统一点初始化时间）
        self.timer = self.create_timer(1.0, self.publish_takeoff_command)

    def publish_takeoff_command(self):
        cmd = VehicleCommand()
        # 起飞命令
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        # 参数说明（与 MAVLink MAV_CMD_NAV_TAKEOFF 对应）：
        # param1: 最小俯仰角（可选，0 表示默认）
        # param2: 空速（可选，0 表示默认）
        # param3: 航向（可选，0 表示当前航向）
        # param4: 保留，设为 0
        # param5: 纬度（0 表示当前位置）
        # param6: 经度（0 表示当前位置）
        # param7: 起飞高度（米），必须设置
        cmd.param1 = 0.0
        cmd.param2 = 0.0
        cmd.param3 = 0.0
        cmd.param4 = 0.0
        cmd.param5 = 0.0
        cmd.param6 = 0.0
        cmd.param7 = 2.0          # 起飞到 2 米高度

        cmd.target_system = 1     # 必须与飞控的 MAV_SYS_ID 一致
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 微秒时间戳

        self.publisher.publish(cmd)
        self.get_logger().info('已发布起飞 (Takeoff) 命令，目标高度 2m')
        # 只发布一次，取消定时器
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()