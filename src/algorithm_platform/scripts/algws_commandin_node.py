#!/usr/bin/env python3
# scripts/algws_commandin_node.py

import json
import time

from dataclasses import dataclass
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =========================================================
# CommandMode
#
# 定义命令的执行模式。
# =========================================================
class CommandMode(Enum):

    # INTERRUPT：
    # 新命令到来时，立即中断当前正在执行的命令。
    INTERRUPT = "interrupt"

    # QUEUE：
    # 新命令进入 FIFO 队列，等待前面的命令完成。
    QUEUE = "queue"


# =========================================================
# CommandState
#
# 定义一个 Command 在 CommandingNode 内部的生命周期。
# =========================================================
class CommandState(Enum):

    # 刚刚收到
    RECEIVED = "received"

    # 已经进入等待队列
    QUEUED = "queued"

    # 当前正在执行
    EXECUTING = "executing"

    # 正常完成
    COMPLETED = "completed"

    # 被其他命令中断
    INTERRUPTED = "interrupted"

    # 命令非法，被拒绝
    REJECTED = "rejected"


# =========================================================
# Command
#
# CommandingNode 内部统一使用的数据结构。
#
# 外部收到的是 JSON，
# 进入 Node 后统一转换成 Command 对象。
# =========================================================
@dataclass
class Command:

    # 标准动作名称
    #
    # 例如：
    # takeoff
    # fly_to
    # rtl
    # land
    action: str

    # 命令唯一 ID
    action_id: str

    # interrupt / queue
    mode: CommandMode

    # 动作参数
    #
    # 例如：
    # {"alt": 20}
    #
    # 或：
    # {"pos": [114.3, 30.5, 20]}
    params: dict

    # 当前状态
    state: CommandState = CommandState.RECEIVED


# =========================================================
# CommandingNode
#
# 职责：
#
# 1. 接收 C 发来的 /action_intent
# 2. 解析 JSON
# 3. 验证 action
# 4. 管理 interrupt / queue
# 5. 管理当前命令
# 6. 管理 FIFO queue
# 7. 将当前命令发送给 ArbitrationNode
#
# 注意：
# CommandingNode 不负责：
# - PX4 控制
# - 飞行算法
# - 动作映射
# - 优先级仲裁
#
# 那些事情属于后面的节点。
# =========================================================
class CommandingNode(Node):

    # ---------------------------------------------------------
    # 同事协议规定的 action 白名单
    #
    # 这里直接按照 action_intent 协议。
    # ---------------------------------------------------------
    VALID_ACTIONS = {
        "takeoff",
        "hover",
        "fly_to",
        "track_vel",
        "rtl",
        "land",
        "orbit",
        "turn",
        "stop",
    }

    def __init__(self):

        super().__init__("algws_commanding")

        # =====================================================
        # 1. 接收 C 的正式 action_intent
        #
        # Topic：
        #     /action_intent
        #
        # 类型：
        #     std_msgs/msg/String
        #
        # C 发来的 JSON 会进入 command_callback()。
        # =====================================================
        self.command_subscriber = self.create_subscription(
            String,
            "/action_intent",
            self.command_callback,
            10,
        )

        # =====================================================
        # 2. 发送命令给 ArbitrationNode
        #
        # Topic：
        #     /algorithm/command/raw
        #
        # 注意：
        # 这里发送的是 JSON，
        # 不是简单的 "takeoff" 字符串。
        #
        # 这样可以保留：
        # action_id
        # action
        # params
        #
        # 这些信息。
        # =====================================================
        self.command_publisher = self.create_publisher(
            String,
            "/algorithm/command/raw",
            10,
        )

        # =====================================================
        # 3. 接收命令完成通知
        #
        # 未来真正执行命令的节点完成任务后，
        # 可以向这个 Topic 发布：
        #
        # {
        #     "action_id": "act_001"
        # }
        #
        # CommandingNode 收到以后，
        # 才会执行 FIFO 队列中的下一个命令。
        #
        # 目前这个接口主要是为后续 Execution Node 准备。
        # =====================================================
        self.complete_subscriber = self.create_subscription(
            String,
            "/algorithm/command/completed",
            self.complete_callback,
            10,
        )

        # =====================================================
        # 4. FIFO Queue
        #
        # Queue 模式的命令全部放在这里。
        # =====================================================
        self.command_queue = deque()

        # =====================================================
        # 5. 当前正在执行的命令
        #
        # 没有命令时：
        #
        #     None
        #
        # 有命令时：
        #
        #     Command(...)
        # =====================================================
        self.current_command = None

        self.get_logger().info(
            "Commanding Node started."
        )

    # =========================================================
    # 接收外部命令
    # =========================================================
    def command_callback(self, msg: String):

        # 获取 ROS String 中的 JSON 字符串
        raw_data = msg.data.strip()

        if not raw_data:
            return

        self.get_logger().info(
            f"Received action_intent: {raw_data}"
        )

        # -----------------------------------------------------
        # 解析 JSON
        # -----------------------------------------------------
        command = self.parse_command(raw_data)

        if command is None:
            return

        # -----------------------------------------------------
        # 检查 command 是否合法
        # -----------------------------------------------------
        if not self.validate_command(command):

            command.state = CommandState.REJECTED

            self.get_logger().error(
                f"Command rejected: {command.action_id}"
            )

            return

        # -----------------------------------------------------
        # 进入调度逻辑
        # -----------------------------------------------------
        self.handle_command(command)

    # =========================================================
    # 解析 JSON
    # =========================================================
    def parse_command(self, data: str):

        try:

            # JSON 字符串 → Python dict
            raw = json.loads(data)

        except json.JSONDecodeError:

            self.get_logger().error(
                "Invalid JSON in /action_intent."
            )

            return None

        # JSON 最外层必须是 object
        if not isinstance(raw, dict):

            self.get_logger().error(
                "Command JSON must be an object."
            )

            return None

        # -----------------------------------------------------
        # action 是必填字段
        # -----------------------------------------------------
        if "action" not in raw:

            self.get_logger().error(
                'Missing required field: "action".'
            )

            return None

        action = raw["action"]

        # action 必须是字符串
        if not isinstance(action, str):

            self.get_logger().error(
                '"action" must be a string.'
            )

            return None

        # -----------------------------------------------------
        # action_id
        #
        # 协议允许 action_id 缺省，
        # 所以 A 可以自动生成。
        # -----------------------------------------------------
        action_id = raw.get(
            "action_id",
            self.generate_action_id(),
        )

        # -----------------------------------------------------
        # params
        #
        # 没有 params 时使用 {}。
        # -----------------------------------------------------
        params = raw.get(
            "params",
            {},
        )

        # params 必须是 dict
        if not isinstance(params, dict):

            self.get_logger().error(
                '"params" must be an object.'
            )

            return None

        # -----------------------------------------------------
        # 正式 C 协议没有 mode 字段。
        #
        # 所以外部 C 命令默认：
        #
        #     INTERRUPT
        #
        # 这与 action_intent “立即执行、可被替换/打断”
        # 的协议语义一致。
        # -----------------------------------------------------
        mode = CommandMode.INTERRUPT

        return Command(
            action=action,
            action_id=action_id,
            mode=mode,
            params=params,
        )

    # =========================================================
    # 验证命令
    # =========================================================
    def validate_command(self, command: Command):

        # -----------------------------------------------------
        # 检查 action 是否在协议白名单里
        # -----------------------------------------------------
        if command.action not in self.VALID_ACTIONS:

            self.get_logger().error(
                f"Unknown action: {command.action}"
            )

            return False

        # -----------------------------------------------------
        # 检查 action_id
        # -----------------------------------------------------
        if not command.action_id:

            self.get_logger().error(
                "action_id cannot be empty."
            )

            return False

        return True

    # =========================================================
    # 调度命令
    # =========================================================
    def handle_command(self, command: Command):

        if command.mode == CommandMode.INTERRUPT:

            self.handle_interrupt(command)

        elif command.mode == CommandMode.QUEUE:

            self.handle_queue(command)

    # =========================================================
    # Interrupt 模式
    # =========================================================
    def handle_interrupt(self, command: Command):

        # -----------------------------------------------------
        # 如果已经有命令执行，
        # 将它标记为 INTERRUPTED。
        # -----------------------------------------------------
        if self.current_command is not None:

            self.current_command.state = (
                CommandState.INTERRUPTED
            )

            self.get_logger().warn(
                f"Command "
                f"{self.current_command.action_id} "
                f"interrupted by "
                f"{command.action_id}"
            )

        # -----------------------------------------------------
        # V1：
        #
        # interrupt 表示：
        # “现在马上执行这个命令。”
        #
        # 所以之前排队的命令全部清掉。
        # -----------------------------------------------------
        self.command_queue.clear()

        # 开始执行新命令
        self.start_command(command)

    # =========================================================
    # Queue 模式
    # =========================================================
    def handle_queue(self, command: Command):

        # 设置状态
        command.state = CommandState.QUEUED

        # 加入 FIFO 队列尾部
        self.command_queue.append(command)

        self.get_logger().info(
            f"Command queued: "
            f"{command.action_id}"
        )

        # 如果现在没有命令执行，
        # 立即执行队列里的第一个。
        if self.current_command is None:

            self.execute_next()

    # =========================================================
    # 开始执行
    # =========================================================
    def start_command(self, command: Command):

        command.state = CommandState.EXECUTING

        # 设置当前命令
        self.current_command = command

        self.get_logger().info(
            f"Executing: "
            f"{command.action} "
            f"[{command.action_id}]"
        )

        # 发布给 ArbitrationNode
        self.publish_current_command(command)

    # =========================================================
    # 从 FIFO 队列中取下一个命令
    # =========================================================
    def execute_next(self):

        # 如果当前还有命令执行，
        # 不允许再启动下一个。
        if self.current_command is not None:
            return

        # 如果队列为空，没有东西执行。
        if not self.command_queue:
            return

        # 从队列头部取出命令。
        #
        # popleft() = FIFO
        command = self.command_queue.popleft()

        self.start_command(command)

    # =========================================================
    # 接收完成通知
    # =========================================================
    def complete_callback(self, msg: String):

        raw_data = msg.data.strip()

        if not raw_data:
            return

        try:

            data = json.loads(raw_data)

        except json.JSONDecodeError:

            self.get_logger().error(
                "Invalid JSON in command completion."
            )

            return

        # 获取完成的 action_id
        action_id = data.get("action_id")

        if not action_id:

            self.get_logger().error(
                'Missing "action_id" in completion message.'
            )

            return

        # 完成当前命令
        self.complete_current_command(action_id)

    # =========================================================
    # 完成当前命令
    # =========================================================
    def complete_current_command(self, action_id):

        # 没有当前命令
        if self.current_command is None:

            self.get_logger().warn(
                f"Completion received for "
                f"{action_id}, "
                f"but no command is executing."
            )

            return

        # -----------------------------------------------------
        # 防止错误的 action_id 把当前命令完成掉。
        # -----------------------------------------------------
        if self.current_command.action_id != action_id:

            self.get_logger().warn(
                f"Completion ID mismatch: "
                f"expected "
                f"{self.current_command.action_id}, "
                f"received {action_id}"
            )

            return

        # 设置完成状态
        self.current_command.state = (
            CommandState.COMPLETED
        )

        self.get_logger().info(
            f"Command completed: {action_id}"
        )

        # 清除当前命令
        self.current_command = None

        # 自动执行 FIFO 中的下一个
        self.execute_next()

    # =========================================================
    # 发布当前命令
    # =========================================================
    def publish_current_command(self, command: Command):

        msg = String()

        # -----------------------------------------------------
        # 这里不要把 JSON 压缩成：
        #
        #     "takeoff"
        #
        # 因为这样会丢掉：
        #
        #     action_id
        #     params
        #
        # C 的正式协议本身就是 JSON，
        # 所以我们把结构完整传给 ArbitrationNode。
        # -----------------------------------------------------
        msg.data = json.dumps(
            {
                "action_id": command.action_id,
                "action": command.action,
                "params": command.params,
            },
            ensure_ascii=False,
        )

        self.command_publisher.publish(msg)

        self.get_logger().info(
            f"Published to ArbitrationNode: "
            f"{msg.data}"
        )

    # =========================================================
    # 生成 action_id
    # =========================================================
    def generate_action_id(self):

        # 当前时间戳，单位毫秒
        timestamp_ms = int(
            time.time() * 1000
        )

        return f"act_{timestamp_ms}"


# =============================================================
# main
# =============================================================
def main(args=None):

    # 初始化 ROS 2
    rclpy.init(args=args)

    # 创建 Node
    node = CommandingNode()

    try:

        # 持续运行 Node
        rclpy.spin(node)

    except KeyboardInterrupt:

        # Ctrl+C
        pass

    finally:

        # 销毁 Node
        node.destroy_node()

        # 关闭 ROS 2
        rclpy.shutdown()


# =============================================================
# Python 程序入口
# =============================================================
if __name__ == "__main__":
    main()