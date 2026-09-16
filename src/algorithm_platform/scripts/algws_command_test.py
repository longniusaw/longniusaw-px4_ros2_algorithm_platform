#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandTestNode(Node):
    def __init__(self):
        super().__init__('algws_command_test')
        self.publisher = self.create_publisher(String, '/algorithm/command/raw', 10)
        self.get_logger().info('📤 测试指令节点已启动，准备发送指令序列...')

        self.timer = self.create_timer(5.0, self.send_command_sequence)

        self.commands = [
            'takeoff',
            'hover',
            'fly_left_5m',
            'fly_forward_5m'
        ]
        self.index = 0

    def send_command_sequence(self):
        if self.index < len(self.commands):
            cmd = self.commands[self.index]
            msg = String()
            msg.data = cmd
            self.publisher.publish(msg)
            self.get_logger().info(f'📤 发送指令: {cmd}')
            self.index += 1
        else:
            self.get_logger().info('✅ 所有指令已发送完毕')
            self.timer.cancel() #发完关闭定时器

def main(args=None):
    rclpy.init(args=args)
    node = CommandTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
