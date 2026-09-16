#!/usr/bin/env python3

import json
from datetime import datetime
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MissionState(Enum):
    """
    Mission 的状态。

    Enum 可以理解成：
    给一些固定的状态起名字，避免我们到处直接写字符串。
    """

    IDLE = "IDLE"
    RECEIVED = "RECEIVED"
    EXECUTING = "EXECUTING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"
    FAILED = "FAILED"


class MissionManager(Node):

    def __init__(self):
        """
        初始化 Mission Manager。
        """

        # 调用父类 Node 的初始化函数。
        # 这里给 ROS 2 节点起名字：algws_mission_manager
        super().__init__("algws_mission_manager")

        # ============================================================
        # 1. 当前 Mission
        # ============================================================

        # 当前正在管理的任务。
        #
        # 如果现在没有任务，就是 None。
        #
        # None 可以简单理解成：
        # “现在没有东西”。
        self.current_mission = None

        # 当前任务状态。
        self.current_state = MissionState.IDLE

        # 当前执行到第几个 waypoint。
        #
        # -1 表示还没有开始执行。
        # 0 表示第一个 waypoint。
        # 1 表示第二个 waypoint。
        self.current_waypoint_index = -1

        # ============================================================
        # 2. Mission History
        # ============================================================

        # 保存已经结束的 Mission。
        #
        # 现在只是一个简单的 Python list。
        # 后面如果项目需要，可以换成数据库或者日志系统。
        self.mission_history = []

        # ============================================================
        # 3. ROS Subscriber
        # ============================================================

        # 监听 C → A 的任务话题。
        #
        # 消息类型：
        # std_msgs/msg/String
        #
        # 例如收到：
        #
        # {
        #   "mission_id": "mission_001",
        #   "mission_type": 1,
        #   "title": "航点任务",
        #   "waypoints": [...]
        # }
        #
        self.mission_subscriber = self.create_subscription(
            String,
            "/mission_intent",
            self.mission_intent_callback,
            10
        )

        # ============================================================
        # 4. Mission Command Publisher
        # ============================================================

        # 给 Mission Executor 发布：
        #
        # “现在这个 Mission 应该执行什么阶段”
        #
        # 例如：
        #
        # {
        #   "mission_id": "mission_001",
        #   "mission_type": 1,
        #   "waypoint_index": 0,
        #   "waypoint": [114.305299, 30.592800, 20]
        # }
        #
        self.command_publisher = self.create_publisher(
            String,
            "/mission_command",
            10
        )

        # ============================================================
        # 5. Mission Status Publisher
        # ============================================================

        # 给其他节点 / C 发布 Mission 当前状态。
        #
        # 以后可以和协议中的 /mission_status 对接。
        self.status_publisher = self.create_publisher(
            String,
            "/mission_status",
            10
        )

        # ============================================================
        # 6. 定时器
        # ============================================================

        # 每 1 秒发布一次 Mission Status。
        #
        # 这和你们协议中：
        #
        # /mission_status
        # A → C → D
        # 1 Hz
        #
        # 的思路保持一致。
        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )

        self.get_logger().info(
            "🚀 Mission Manager started."
        )

    # =================================================================
    # Mission Intent
    # =================================================================

    def mission_intent_callback(self, msg: String):
        """
        收到 /mission_intent 后执行。

        msg.data 是字符串。

        例如：
        '{"mission_id":"mission_001", ...}'
        """

        self.get_logger().info(
            f"📥 Received mission intent: {msg.data}"
        )

        # -------------------------------------------------------------
        # 第一步：把 JSON 字符串转换成 Python 字典
        # -------------------------------------------------------------

        try:
            mission = json.loads(msg.data)

        except json.JSONDecodeError as error:
            self.get_logger().error(
                f"❌ Invalid JSON: {error}"
            )
            return

        # -------------------------------------------------------------
        # 第二步：验证 Mission
        # -------------------------------------------------------------

        if not self.validate_mission(mission):
            self.get_logger().error(
                "❌ Mission validation failed."
            )
            return

        # -------------------------------------------------------------
        # 第三步：如果当前已经有 Mission
        # -------------------------------------------------------------

        if self.current_mission is not None:

            self.get_logger().warn(
                f"⚠️ Current mission "
                f"{self.current_mission.get('mission_id')} "
                f"will be replaced."
            )

            # 把旧任务记录下来。
            self.finish_current_mission(
                MissionState.CANCELLED
            )

        # -------------------------------------------------------------
        # 第四步：保存新的 Mission
        # -------------------------------------------------------------

        self.current_mission = mission

        # 新任务刚刚收到。
        self.current_state = MissionState.RECEIVED

        # 默认还没有开始执行 waypoint。
        self.current_waypoint_index = -1

        mission_id = mission["mission_id"]

        self.get_logger().info(
            f"✅ Mission accepted: {mission_id}"
        )

        # -------------------------------------------------------------
        # 第五步：根据 Mission Type 处理
        # -------------------------------------------------------------

        mission_type = mission["mission_type"]

        if mission_type == 1:

            self.prepare_waypoint_mission()

        else:

            self.get_logger().error(
                f"❌ Unsupported mission_type: {mission_type}"
            )

            self.current_state = MissionState.FAILED

    # =================================================================
    # Mission Validation
    # =================================================================

    def validate_mission(self, mission):
        """
        检查 Mission 是否符合我们当前协议。

        返回：
            True  -> 合法
            False -> 非法
        """

        # -------------------------------------------------------------
        # 检查 mission_id
        # -------------------------------------------------------------

        if "mission_id" not in mission:

            self.get_logger().error(
                "Mission missing 'mission_id'."
            )

            return False

        if not mission["mission_id"]:

            self.get_logger().error(
                "Mission 'mission_id' is empty."
            )

            return False

        # -------------------------------------------------------------
        # 检查 mission_type
        # -------------------------------------------------------------

        if "mission_type" not in mission:

            self.get_logger().error(
                "Mission missing 'mission_type'."
            )

            return False

        # -------------------------------------------------------------
        # 当前协议：
        #
        # mission_type = 1
        # 表示 waypoint mission
        #
        # -------------------------------------------------------------

        if mission["mission_type"] == 1:

            if "waypoints" not in mission:

                self.get_logger().error(
                    "Waypoint mission missing 'waypoints'."
                )

                return False

            if not isinstance(
                mission["waypoints"],
                list
            ):

                self.get_logger().error(
                    "'waypoints' must be a list."
                )

                return False

            if len(mission["waypoints"]) == 0:

                self.get_logger().error(
                    "Waypoint list is empty."
                )

                return False

        return True

    # =================================================================
    # Waypoint Mission
    # =================================================================

    def prepare_waypoint_mission(self):
        """
        准备一个 waypoint mission。

        注意：

        这里还没有真正让飞机飞。

        我们只是告诉 Mission Manager：

        “这是一个航点任务，我准备开始执行。”
        """

        waypoints = self.current_mission["waypoints"]

        self.get_logger().info(
            f"🗺️ Waypoint mission contains "
            f"{len(waypoints)} waypoints."
        )

        # 第一个 waypoint。
        self.current_waypoint_index = 0

        # Mission 正式进入执行状态。
        self.current_state = MissionState.EXECUTING

        # 给 Mission Executor 发布：
        #
        # “执行第 0 个 waypoint”
        self.publish_current_mission_command()

    # =================================================================
    # Publish Mission Command
    # =================================================================

    def publish_current_mission_command(self):
        """
        发布当前 Mission 的执行信息。

        这个消息给 Mission Executor。

        Mission Manager 不负责决定具体怎么飞。
        """

        if self.current_mission is None:

            return

        mission_type = self.current_mission["mission_type"]

        # =============================================================
        # Waypoint Mission
        # =============================================================

        if mission_type == 1:

            waypoints = self.current_mission["waypoints"]

            # 防止 waypoint index 越界。
            if (
                self.current_waypoint_index < 0
                or
                self.current_waypoint_index >= len(waypoints)
            ):

                self.get_logger().error(
                    "Waypoint index out of range."
                )

                self.current_state = MissionState.FAILED

                return

            waypoint = waypoints[
                self.current_waypoint_index
            ]

            command = {
                "mission_id":
                    self.current_mission["mission_id"],

                "mission_type":
                    mission_type,

                "waypoint_index":
                    self.current_waypoint_index,

                "waypoint":
                    waypoint,

                "command":
                    "fly_to"
            }

            self.publish_json(
                self.command_publisher,
                command
            )

            self.get_logger().info(
                f"🎯 Mission Executor command: "
                f"WP {self.current_waypoint_index} "
                f"→ {waypoint}"
            )

    # =================================================================
    # Waypoint Completed
    # =================================================================

    def waypoint_completed(self):
        """
        告诉 Mission Manager：

        当前 waypoint 已经完成。

        注意：

        现在这个函数还没有 ROS Topic 接入口。

        下一版我们可以增加：

        /mission_executor/status

        让 Mission Executor 告诉 Mission Manager：
        “WP0 到了。”
        """

        if self.current_mission is None:

            return

        if self.current_mission["mission_type"] != 1:

            return

        waypoints = self.current_mission["waypoints"]

        # -------------------------------------------------------------
        # 判断是不是最后一个 waypoint
        # -------------------------------------------------------------

        if (
            self.current_waypoint_index
            >= len(waypoints) - 1
        ):

            # 所有 waypoint 都完成。
            self.current_state = MissionState.COMPLETED

            self.get_logger().info(
                "✅ Mission completed."
            )

            self.finish_current_mission(
                MissionState.COMPLETED
            )

            return

        # -------------------------------------------------------------
        # 还有下一个 waypoint
        # -------------------------------------------------------------

        self.current_waypoint_index += 1

        self.get_logger().info(
            f"➡️ Moving to waypoint "
            f"{self.current_waypoint_index}"
        )

        self.publish_current_mission_command()

    # =================================================================
    # Cancel Mission
    # =================================================================

    def cancel_current_mission(self):
        """
        取消当前 Mission。
        """

        if self.current_mission is None:

            self.get_logger().warn(
                "No active mission to cancel."
            )

            return

        mission_id = self.current_mission["mission_id"]

        self.get_logger().warn(
            f"🛑 Mission cancelled: {mission_id}"
        )

        self.finish_current_mission(
            MissionState.CANCELLED
        )

    # =================================================================
    # Finish Mission
    # =================================================================

    def finish_current_mission(self, final_state):
        """
        结束当前 Mission，并把它放进 history。
        """

        if self.current_mission is None:

            return

        mission_record = {

            "mission_id":
                self.current_mission["mission_id"],

            "mission_type":
                self.current_mission["mission_type"],

            "title":
                self.current_mission.get(
                    "title",
                    ""
                ),

            "final_state":
                final_state.value,

            "finished_at":
                self.now_string()
        }

        self.mission_history.append(
            mission_record
        )

        self.get_logger().info(
            f"📝 Mission history updated: "
            f"{mission_record}"
        )

        # 清除当前 Mission。
        self.current_mission = None

        self.current_state = MissionState.IDLE

        self.current_waypoint_index = -1

    # =================================================================
    # Publish Status
    # =================================================================

    def publish_status(self):
        """
        每秒发布一次 Mission Status。

        目前先发布我们 A Edge 能确定的信息。

        后面可以逐渐加入：
        progress
        pos_ned
        battery_pct
        target_visible
        plan_id
        alarm
        debug
        history
        """

        status = {

            "mission_id":
                (
                    self.current_mission["mission_id"]
                    if self.current_mission
                    else ""
                ),

            "state":
                self.current_state.value,

            "state_str":
                self.current_state.value,

            "progress":
                self.calculate_progress(),

            "current_waypoint":
                self.current_waypoint_index,

            "history":
                self.mission_history[-10:]
        }

        self.publish_json(
            self.status_publisher,
            status
        )

    # =================================================================
    # Calculate Progress
    # =================================================================

    def calculate_progress(self):
        """
        计算 Mission 当前进度。

        返回 0.0 ~ 1.0。
        """

        if self.current_mission is None:

            return 0.0

        if self.current_mission["mission_type"] != 1:

            return 0.0

        waypoints = self.current_mission["waypoints"]

        if len(waypoints) == 0:

            return 0.0

        # 如果只有一个 waypoint：
        #
        # 开始执行时：
        # index = 0
        #
        # 这里先认为执行到第一个 waypoint
        # 就是 0.0。
        if len(waypoints) == 1:

            if self.current_state == MissionState.COMPLETED:

                return 1.0

            return 0.0

        progress = (
            self.current_waypoint_index
            /
            (len(waypoints) - 1)
        )

        # 防止出现 < 0 或 > 1。
        return max(
            0.0,
            min(
                1.0,
                progress
            )
        )

    # =================================================================
    # JSON Publisher Helper
    # =================================================================

    def publish_json(self, publisher, data):
        """
        一个通用函数：

        Python dict
            ↓
        JSON 字符串
            ↓
        ROS String
            ↓
        publish()
        """

        msg = String()

        msg.data = json.dumps(
            data,
            ensure_ascii=False
        )

        publisher.publish(msg)

    # =================================================================
    # Time Helper
    # =================================================================

    def now_string(self):
        """
        获取当前时间。

        返回类似：

        2026-09-14T13:30:25
        """

        return datetime.now().isoformat(
            timespec="seconds"
        )


def main(args=None):

    # 初始化 ROS 2。
    rclpy.init(args=args)

    # 创建 Mission Manager。
    node = MissionManager()

    try:

        # 进入 ROS 2 事件循环。
        #
        # 只要程序不退出：
        #
        # - 收到 topic
        # - timer 到时间
        # - callback
        #
        # 都会在这里被处理。
        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # 销毁节点。
        node.destroy_node()

        # 关闭 ROS 2。
        rclpy.shutdown()


if __name__ == "__main__":
    main()