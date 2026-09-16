#!/usr/bin/env python3

import json
import time

from dataclasses import dataclass
from enum import Enum
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =========================================================
# Command Mode
# =========================================================

class CommandMode(Enum):

    INTERRUPT = "interrupt"

    QUEUE = "queue"


# =========================================================
# Command State
# =========================================================

class CommandState(Enum):

    RECEIVED = "RECEIVED"

    QUEUED = "QUEUED"

    EXECUTING = "EXECUTING"

    COMPLETED = "COMPLETED"

    FAILED = "FAILED"

    CANCELLED = "CANCELLED"

    REJECTED = "REJECTED"


# =========================================================
# Command
# =========================================================

@dataclass
class Command:

    action: str

    action_id: str

    mode: CommandMode

    params: dict

    state: CommandState = CommandState.RECEIVED


# =========================================================
# Commanding Node
# =========================================================

class CommandingNode(Node):

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
        # /action_intent
        # =====================================================

        self.command_subscriber = self.create_subscription(
            String,
            "/action_intent",
            self.command_callback,
            10,
        )

        # =====================================================
        # Command → Arbitration
        # =====================================================

        self.command_publisher = self.create_publisher(
            String,
            "/algorithm/command/raw",
            10,
        )

        # =====================================================
        # Execution control → Communication
        # =====================================================

        self.control_publisher = self.create_publisher(
            String,
            "/algorithm/command/control",
            10,
        )

        # =====================================================
        # Communication → Commanding
        # =====================================================

        self.status_subscriber = self.create_subscription(
            String,
            "/algorithm/command/status",
            self.status_callback,
            10,
        )

        # =====================================================
        # FIFO
        # =====================================================

        self.command_queue = deque()

        # =====================================================
        # Current command
        # =====================================================

        self.current_command = None

        self.get_logger().info(
            "Commanding Node started."
        )

    # =========================================================
    # Receive action_intent
    # =========================================================

    def command_callback(self, msg: String):

        raw_data = msg.data.strip()

        if not raw_data:
            return

        self.get_logger().info(
            f"Received action_intent: {raw_data}"
        )

        command = self.parse_command(raw_data)

        if command is None:
            return

        if not self.validate_command(command):

            command.state = CommandState.REJECTED

            self.get_logger().error(
                f"❌ Command rejected: "
                f"{command.action_id}"
            )

            return

        self.handle_command(command)

    # =========================================================
    # Parse
    # =========================================================

    def parse_command(self, data: str):

        try:

            raw = json.loads(data)

        except json.JSONDecodeError as exc:

            self.get_logger().error(
                f"❌ Invalid JSON: {exc}"
            )

            return None

        if not isinstance(raw, dict):

            self.get_logger().error(
                "❌ Command JSON must be an object."
            )

            return None

        # -----------------------------------------------------
        # action
        # -----------------------------------------------------

        action = raw.get("action")

        if not isinstance(action, str):

            self.get_logger().error(
                '❌ "action" must be a string.'
            )

            return None

        # -----------------------------------------------------
        # action_id
        # -----------------------------------------------------

        action_id = raw.get(
            "action_id",
            self.generate_action_id(),
        )

        if not isinstance(action_id, str):

            action_id = str(action_id)

        # -----------------------------------------------------
        # params
        # -----------------------------------------------------

        params = raw.get(
            "params",
            {},
        )

        if not isinstance(params, dict):

            self.get_logger().error(
                '❌ "params" must be an object.'
            )

            return None

        # -----------------------------------------------------
        # mode
        #
        # 正式同事协议没有强制要求 mode。
        #
        # 所以：
        #
        # 外部 C → 默认 interrupt
        #
        # 但为了你自己的 debug，
        # 可以额外发送：
        #
        # "mode": "queue"
        # -----------------------------------------------------

        mode_value = raw.get(
            "mode",
            "interrupt",
        )

        if mode_value == "queue":

            mode = CommandMode.QUEUE

        else:

            mode = CommandMode.INTERRUPT

        return Command(
            action=action,
            action_id=action_id,
            mode=mode,
            params=params,
        )

    # =========================================================
    # Validation
    # =========================================================

    def validate_command(self, command: Command):

        if command.action not in self.VALID_ACTIONS:

            self.get_logger().error(
                f"❌ Unknown action: "
                f"{command.action}"
            )

            return False

        if not command.action_id:

            self.get_logger().error(
                "❌ action_id cannot be empty."
            )

            return False

        return True

    # =========================================================
    # Scheduling
    # =========================================================

    def handle_command(self, command: Command):

        if command.mode == CommandMode.INTERRUPT:

            self.handle_interrupt(command)

        else:

            self.handle_queue(command)

    # =========================================================
    # Interrupt
    # =========================================================

    def handle_interrupt(self, command: Command):

        # -----------------------------------------------------
        # 1. Cancel currently executing command
        # -----------------------------------------------------

        if self.current_command is not None:

            old_command = self.current_command

            old_command.state = (
                CommandState.CANCELLED
            )

            self.get_logger().warn(
                f"⚠️ Command "
                f"{old_command.action_id} "
                f"cancelled by "
                f"{command.action_id}"
            )

            self.publish_cancel(
                old_command.action_id
            )

            self.current_command = None

        # -----------------------------------------------------
        # 2. Clear FIFO
        # -----------------------------------------------------

        self.command_queue.clear()

        # -----------------------------------------------------
        # 3. Start new command
        # -----------------------------------------------------

        self.start_command(command)

    # =========================================================
    # Queue
    # =========================================================

    def handle_queue(self, command: Command):

        command.state = CommandState.QUEUED

        self.command_queue.append(command)

        self.get_logger().info(
            f"📦 Command queued: "
            f"{command.action_id}"
        )

        if self.current_command is None:

            self.execute_next()

    # =========================================================
    # Start command
    # =========================================================

    def start_command(self, command: Command):

        command.state = CommandState.EXECUTING

        self.current_command = command

        self.get_logger().info(
            f"▶️ Executing: "
            f"{command.action} "
            f"[{command.action_id}]"
        )

        self.publish_command(command)

    # =========================================================
    # Next command
    # =========================================================

    def execute_next(self):

        if self.current_command is not None:
            return

        if not self.command_queue:
            return

        command = self.command_queue.popleft()

        self.start_command(command)

    # =========================================================
    # Publish command
    # =========================================================

    def publish_command(self, command: Command):

        msg = String()

        # -----------------------------------------------------
        # 关键：
        #
        # 不发送 mode。
        #
        # 保持同事协议：
        #
        # action_id
        # action
        # params
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
            f"📤 Published to ArbitrationNode: "
            f"{msg.data}"
        )

    # =========================================================
    # Cancel current command
    # =========================================================

    def publish_cancel(self, action_id):

        msg = String()

        msg.data = json.dumps(
            {
                "action_id": action_id,
                "command": "cancel",
            },
            ensure_ascii=False,
        )

        self.control_publisher.publish(msg)

        self.get_logger().info(
            f"🛑 Cancel sent to Communication: "
            f"{msg.data}"
        )

    # =========================================================
    # Execution status
    # =========================================================

    def status_callback(self, msg: String):

        raw_data = msg.data.strip()

        if not raw_data:
            return

        try:

            data = json.loads(raw_data)

        except json.JSONDecodeError:

            self.get_logger().error(
                "❌ Invalid command status JSON."
            )

            return

        action_id = data.get("action_id")

        state = data.get("state")

        if not action_id or not state:

            self.get_logger().error(
                "❌ Invalid command status."
            )

            return

        self.get_logger().info(
            f"📡 Execution status: "
            f"{action_id} -> {state}"
        )

        # -----------------------------------------------------
        # Ignore status from an old command
        # -----------------------------------------------------

        if (
            self.current_command is None
            or
            self.current_command.action_id != action_id
        ):

            self.get_logger().debug(
                f"Ignore status for "
                f"{action_id}"
            )

            return

        # -----------------------------------------------------
        # EXECUTING
        # -----------------------------------------------------

        if state == "EXECUTING":

            self.current_command.state = (
                CommandState.EXECUTING
            )

            return

        # -----------------------------------------------------
        # COMPLETED
        # -----------------------------------------------------

        if state == "COMPLETED":

            self.current_command.state = (
                CommandState.COMPLETED
            )

            self.get_logger().info(
                f"✅ Command completed: "
                f"{action_id}"
            )

            self.current_command = None

            self.execute_next()

            return

        # -----------------------------------------------------
        # FAILED
        # -----------------------------------------------------

        if state == "FAILED":

            self.current_command.state = (
                CommandState.FAILED
            )

            reason = data.get(
                "reason",
                "unknown",
            )

            self.get_logger().error(
                f"❌ Command failed: "
                f"{action_id}, "
                f"reason={reason}"
            )

            self.current_command = None

            self.execute_next()

            return

        # -----------------------------------------------------
        # CANCELLED
        # -----------------------------------------------------

        if state == "CANCELLED":

            self.current_command.state = (
                CommandState.CANCELLED
            )

            self.get_logger().warn(
                f"🛑 Command cancelled: "
                f"{action_id}"
            )

            self.current_command = None

            self.execute_next()

            return

    # =========================================================
    # Generate action ID
    # =========================================================

    def generate_action_id(self):

        timestamp_ms = int(
            time.time() * 1000
        )

        return f"act_{timestamp_ms}"


# =============================================================
# main
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = CommandingNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()