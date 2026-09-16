#!/usr/bin/env python3
# scripts/algws_arbitration_node.py

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ArbitrationNode(Node):

    def __init__(self):

        super().__init__("algws_arbitration")

        # =====================================================
        # 输入 Topic
        #
        # CommandingNode
        #       ↓
        # /algorithm/command/raw
        #
        # 输入类型：
        # std_msgs/msg/String
        #
        # String.data 中保存 JSON。
        # =====================================================
        self.create_subscription(
            String,
            "/algorithm/command/raw",
            self.command_callback,
            10,
        )

        # =====================================================
        # 输出 Topic
        #
        # ArbitrationNode
        #       ↓
        # /algorithm/command/arbitrated
        #
        # 这里发布最终仲裁后的命令。
        # =====================================================
        self.publisher = self.create_publisher(
            String,
            "/algorithm/command/arbitrated",
            10,
        )

        # =====================================================
        # 优先级表
        #
        # 数字越小，优先级越高。
        #
        # 注意：
        # 当前 V1 先保留这个表。
        # 真正的“多个命令竞争控制权”逻辑，
        # 后续再实现。
        # =====================================================
        self.priority_map = {

            "emergency_stop": 0,

            "land": 1,

            "takeoff": 2,

            "fly_relative": 3,

            "hover": 4,
        }

        self.get_logger().info(
            "⚖️ Arbitration Node started."
        )

    # =========================================================
    # 接收 CommandingNode 的命令
    # =========================================================
    def command_callback(self, msg: String):

        raw_data = msg.data.strip()

        if not raw_data:
            return

        self.get_logger().info(
            f"📥 收到 CommandingNode 指令: "
            f"{raw_data}"
        )

        # -----------------------------------------------------
        # 解析 JSON
        # -----------------------------------------------------
        try:

            command = json.loads(raw_data)

        except json.JSONDecodeError:

            self.get_logger().error(
                "❌ 无法解析 command JSON."
            )

            return

        # JSON 必须是 object
        if not isinstance(command, dict):

            self.get_logger().error(
                "❌ Command must be a JSON object."
            )

            return

        # -----------------------------------------------------
        # 获取 action
        # -----------------------------------------------------
        action = command.get("action")

        if not action:

            self.get_logger().error(
                '❌ Missing "action".'
            )

            return

        # -----------------------------------------------------
        # 获取 params
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
        # 获取 action_id
        #
        # 仲裁后的最终动作目前不改变原来的动作格式，
        # action_id 主要用于日志和后续命令追踪。
        # -----------------------------------------------------
        action_id = command.get(
            "action_id",
            "unknown",
        )

        # =====================================================
        # 执行动作映射
        # =====================================================
        processed = self._parse_command(
            action,
            params,
        )

        if processed is None:
            return

        # =====================================================
        # 发布最终命令
        #
        # 注意：
        # 这里保持原 ArbitrationNode 的最终输出逻辑。
        #
        # 例如：
        #
        # takeoff
        #     ↓
        # {"action": "takeoff", "altitude": 2.0}
        #
        # fly_forward_5m
        #     ↓
        # {
        #     "action": "fly_relative",
        #     "dx": 5.0,
        #     "dy": 0.0,
        #     "dz": 0.0
        # }
        # =====================================================
        out_msg = String()

        out_msg.data = json.dumps(
            processed,
            ensure_ascii=False,
        )

        self.publisher.publish(out_msg)

        self.get_logger().info(
            f"📤 发布最终仲裁指令 "
            f"[{action_id}]: "
            f"{processed}"
        )

    # =========================================================
    # 动作映射
    #
    # 这是你原来 ArbitrationNode 的核心逻辑。
    #
    # 这里保留原来的测试命令。
    # =========================================================
    def _parse_command(
        self,
        action,
        params,
    ):

        # =====================================================
        # 1. 原来的测试动作
        #
        # 这些最终输出保持不变。
        # =====================================================

        if action == "takeoff":

            # 原测试版本：
            #
            # "takeoff":
            # {
            #     "action": "takeoff",
            #     "altitude": 2.0
            # }
            #
            # 保持原逻辑。
            return {
                "action": "takeoff",
                "altitude": 2.0,
            }

        if action == "hover":

            # 原测试版本：
            #
            # {"action": "hover"}
            return {
                "action": "hover",
            }

        if action == "land":

            # 原测试版本：
            #
            # {"action": "land"}
            return {
                "action": "land",
            }

        # =====================================================
        # 2. 相对飞行测试动作
        #
        # 这些也是原来的逻辑。
        # =====================================================

        if action == "fly_left_5m":

            return {
                "action": "fly_relative",
                "dx": 0.0,
                "dy": 5.0,
                "dz": 0.0,
            }

        if action == "fly_forward_5m":

            return {
                "action": "fly_relative",
                "dx": 5.0,
                "dy": 0.0,
                "dz": 0.0,
            }

        if action == "fly_right_5m":

            return {
                "action": "fly_relative",
                "dx": 0.0,
                "dy": -5.0,
                "dz": 0.0,
            }

        if action == "fly_backward_5m":

            return {
                "action": "fly_relative",
                "dx": -5.0,
                "dy": 0.0,
                "dz": 0.0,
            }

        # =====================================================
        # 3. 同事协议中的标准动作
        #
        # 这些动作目前你的测试 ArbitrationNode
        # 没有定义具体的转换规则。
        #
        # 因此这里不擅自改变其语义。
        #
        # 保留：
        #
        #     action
        #     params
        #
        # 后面的真正执行层再解释。
        # =====================================================

        if action in {
            "fly_to",
            "track_vel",
            "rtl",
            "orbit",
            "turn",
            "stop",
        }:

            result = {
                "action": action,
            }

            # 如果有参数，则原样保留。
            if params:

                result.update(params)

            return result

        # =====================================================
        # 4. 未知动作
        # =====================================================

        self.get_logger().warn(
            f"⚠️ 未知指令: {action}"
        )

        return None


# =============================================================
# main
# =============================================================
def main(args=None):

    # 初始化 ROS 2
    rclpy.init(args=args)

    # 创建 ArbitrationNode
    node = ArbitrationNode()

    try:

        # 持续运行
        rclpy.spin(node)

    except KeyboardInterrupt:

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