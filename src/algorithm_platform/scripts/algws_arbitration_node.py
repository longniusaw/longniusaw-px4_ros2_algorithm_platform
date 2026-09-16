#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArbitrationNode(Node):

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
        super().__init__("algws_arbitration")

        # =====================================================
        # Input
        # =====================================================

        self.create_subscription(
            String,
            "/algorithm/command/raw",
            self.command_callback,
            10,
        )

        # =====================================================
        # Output
        # =====================================================

        self.publisher = self.create_publisher(
            String,
            "/algorithm/command/arbitrated",
            10,
        )

        # =====================================================
        # Priority
        #
        # 数字越小，优先级越高。
        #
        # 当前版本主要保存优先级定义，
        # 不主动改变 Commanding 已经做出的调度。
        # =====================================================

        self.priority_map = {
            "stop": 0,
            "land": 1,
            "rtl": 2,
            "takeoff": 3,
            "fly_to": 4,
            "track_vel": 5,
            "orbit": 6,
            "turn": 7,
            "hover": 8,
        }

        self.get_logger().info(
            "⚖️ Arbitration Node started."
        )

    # =========================================================
    # Command callback
    # =========================================================

    def command_callback(self, msg: String):

        raw_data = msg.data.strip()

        if not raw_data:
            return

        self.get_logger().info(
            f"📥 收到 CommandingNode 指令: {raw_data}"
        )

        # -----------------------------------------------------
        # JSON
        # -----------------------------------------------------

        try:
            command = json.loads(raw_data)

        except json.JSONDecodeError as exc:

            self.get_logger().error(
                f"❌ Command JSON 解析失败: {exc}"
            )

            return

        if not isinstance(command, dict):

            self.get_logger().error(
                "❌ Command 必须是 JSON object."
            )

            return

        # -----------------------------------------------------
        # action
        # -----------------------------------------------------

        action = command.get("action")

        if not isinstance(action, str) or not action:

            self.get_logger().error(
                '❌ Missing or invalid "action".'
            )

            return

        # -----------------------------------------------------
        # action_id
        # -----------------------------------------------------

        action_id = command.get(
            "action_id",
            "unknown",
        )

        # -----------------------------------------------------
        # params
        # -----------------------------------------------------

        params = command.get(
            "params",
            {},
        )

        if not isinstance(params, dict):

            self.get_logger().error(
                '❌ "params" must be an object.'
            )

            return

        # -----------------------------------------------------
        # action validation
        # -----------------------------------------------------

        if action not in self.VALID_ACTIONS:

            self.get_logger().error(
                f"❌ Unsupported action: {action}"
            )

            return

        # =====================================================
        # Arbitration V2
        #
        # 重要：
        #
        # 不再修改：
        #
        #     action
        #     action_id
        #     params
        #
        # 原样传给 Communication。
        # =====================================================

        processed = {
            "action_id": action_id,
            "action": action,
            "params": params,
        }

        # =====================================================
        # Publish
        # =====================================================

        out_msg = String()

        out_msg.data = json.dumps(
            processed,
            ensure_ascii=False,
        )

        self.publisher.publish(out_msg)

        self.get_logger().info(
            f"📤 发布最终仲裁指令 "
            f"[{action_id}]: {out_msg.data}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = ArbitrationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()