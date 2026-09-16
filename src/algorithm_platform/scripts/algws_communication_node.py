#!/usr/bin/env python3

import json
import math

from collections import deque

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
    VehicleGlobalPosition,
)


class CommunicationNode(Node):

    def __init__(self):

        super().__init__(
            "algws_communication"
        )

        # =====================================================
        # PX4 QoS
        # =====================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =====================================================
        # Command input
        # =====================================================

        self.create_subscription(
            String,
            "/algorithm/command/arbitrated",
            self.command_callback,
            10,
        )

        # =====================================================
        # Control input
        #
        # cancel / interrupt
        # =====================================================

        self.create_subscription(
            String,
            "/algorithm/command/control",
            self.control_callback,
            10,
        )

        # =====================================================
        # Execution status
        # =====================================================

        self.status_publisher = self.create_publisher(
            String,
            "/algorithm/command/status",
            10,
        )

        # =====================================================
        # PX4 publishers
        # =====================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        # =====================================================
        # PX4 subscribers
        # =====================================================

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            px4_qos,
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            px4_qos,
        )

        self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.global_position_callback,
            px4_qos,
        )

        # =====================================================
        # Vehicle state
        # =====================================================

        self.armed = False

        self.nav_state = (
            VehicleStatus.NAVIGATION_STATE_MAX
        )

        # Local NED
        #
        # x = North
        # y = East
        # z = Down

        self.current = [
            0.0,
            0.0,
            0.0,
        ]

        self.target = [
            0.0,
            0.0,
            0.0,
        ]

        # Global position
        #
        # latitude / longitude: degrees
        # altitude: meters

        self.global_valid = False

        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

        # =====================================================
        # Command queue
        # =====================================================

        self.command_queue = deque()

        self.current_command = None

        # =====================================================
        # State machine
        # =====================================================

        self.state = "IDLE"

        # =====================================================
        # Offboard
        # =====================================================

        self.offboard_counter = 0

        # =====================================================
        # Takeoff
        # =====================================================

        self.takeoff_tolerance = 0.1

        # =====================================================
        # Position
        # =====================================================

        self.position_tolerance = 0.2

        # =====================================================
        # Orbit
        # =====================================================

        self.orbit_center_lat = 0.0
        self.orbit_center_lon = 0.0
        self.orbit_center_alt = 0.0

        self.orbit_radius = 0.0
        self.orbit_speed = 0.0

        self.orbit_angle = 0.0
        self.orbit_initialized = False

        # =====================================================
        # Timers
        # =====================================================

        # Offboard heartbeat
        self.create_timer(
            0.05,
            self.publish_offboard_mode,
        )

        # Trajectory
        self.create_timer(
            0.02,
            self.publish_setpoint,
        )

        # State machine
        self.create_timer(
            0.05,
            self.state_machine,
        )

        self.get_logger().info(
            "🚀 Communication node started"
        )

    # =========================================================
    # Command callback
    # =========================================================

    def command_callback(self, msg: String):

        try:

            data = json.loads(msg.data)

        except json.JSONDecodeError as exc:

            self.get_logger().error(
                f"❌ JSON解析失败: {exc}"
            )

            return

        if not isinstance(data, dict):

            self.get_logger().error(
                "❌ command must be object"
            )

            return

        action_id = data.get(
            "action_id"
        )

        action = data.get(
            "action"
        )

        params = data.get(
            "params",
            {},
        )

        if not action_id:

            self.get_logger().warn(
                "⚠️ command missing action_id"
            )

            return

        if not action:

            self.get_logger().warn(
                "⚠️ command missing action"
            )

            return

        if not isinstance(params, dict):

            self.publish_failed(
                action_id,
                "INVALID_PARAMS",
            )

            return

        self.get_logger().info(
            f"📥 command received: "
            f"{data}"
        )

        # =====================================================
        # STOP
        #
        # stop 是一个正式动作，
        # 不是普通 cancel。
        # =====================================================

        if action == "stop":

            self.command_queue.clear()

            if self.current_command is not None:

                old_id = self.current_command.get(
                    "action_id"
                )

                self.cancel_current_command(
                    publish_status=True
                )

                self.get_logger().warn(
                    f"🛑 STOP current command: "
                    f"{old_id}"
                )

            # stop 自己进入 hover
            self.current_command = data

            self.target = [
                self.current[0],
                self.current[1],
                self.current[2],
            ]

            self.state = "STOP_HOVER"

            self.publish_executing(
                action_id
            )

            return

        # =====================================================
        # Normal command
        # =====================================================

        self.command_queue.append(data)

        self.get_logger().info(
            f"📦 command queued: "
            f"{data}"
        )

    # =========================================================
    # Control callback
    #
    # Commanding interrupt → Communication
    # =========================================================

    def control_callback(self, msg: String):

        try:

            data = json.loads(msg.data)

        except json.JSONDecodeError:

            self.get_logger().error(
                "❌ Invalid control JSON"
            )

            return

        command = data.get(
            "command"
        )

        action_id = data.get(
            "action_id"
        )

        if command != "cancel":

            return

        if not action_id:

            return

        self.get_logger().warn(
            f"🛑 Cancel request received: "
            f"{action_id}"
        )

        # -----------------------------------------------------
        # Cancel current command
        # -----------------------------------------------------

        if (
            self.current_command is not None
            and
            self.current_command.get(
                "action_id"
            ) == action_id
        ):

            self.cancel_current_command(
                publish_status=True
            )

            return

        # -----------------------------------------------------
        # Cancel queued command
        # -----------------------------------------------------

        new_queue = deque()

        for command_data in self.command_queue:

            if (
                command_data.get(
                    "action_id"
                )
                != action_id
            ):

                new_queue.append(
                    command_data
                )

        self.command_queue = new_queue

    # =========================================================
    # State machine
    # =========================================================

    def state_machine(self):

        # =====================================================
        # IDLE
        # =====================================================

        if self.state == "IDLE":

            if self.current_command is None:

                if not self.command_queue:

                    return

                self.current_command = (
                    self.command_queue.popleft()
                )

                self.start_current_command()

            return

        # =====================================================
        # PREPARE
        # =====================================================

        if self.state == "PREPARE":

            if self.offboard_counter >= 40:

                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )

                self.get_logger().info(
                    "🔄 switch OFFBOARD"
                )

                self.state = (
                    "WAIT_OFFBOARD"
                )

            return

        # =====================================================
        # WAIT OFFBOARD
        # =====================================================

        if self.state == "WAIT_OFFBOARD":

            if (
                self.nav_state
                ==
                VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):

                self.get_logger().info(
                    "✅ OFFBOARD OK"
                )

                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                )

                self.state = "WAIT_ARM"

            return

        # =====================================================
        # WAIT ARM
        # =====================================================

        if self.state == "WAIT_ARM":

            if self.armed:

                self.get_logger().info(
                    "✅ ARM OK"
                )

                self.state = "TAKEOFF"

            return

        # =====================================================
        # TAKEOFF
        # =====================================================

        if self.state == "TAKEOFF":

            altitude = -self.current[2]

            target_altitude = -self.target[2]

            error = abs(
                altitude
                -
                target_altitude
            )

            if (
                error
                <=
                self.takeoff_tolerance
            ):

                self.get_logger().info(
                    f"🛫 TAKEOFF COMPLETE "
                    f"height={altitude:.2f}m"
                )

                action_id = (
                    self.current_command
                    .get("action_id")
                )

                self.publish_completed(
                    action_id
                )

                self.current_command = None

                self.state = "IDLE"

            return

        # =====================================================
        # HOVER
        # =====================================================

        if self.state == "HOVER":

            if (
                self.current_command
                is not None
            ):

                action_id = (
                    self.current_command
                    .get("action_id")
                )

                self.publish_completed(
                    action_id
                )

                self.current_command = None

            self.state = "IDLE"

            return

        # =====================================================
        # STOP HOVER
        # =====================================================

        if self.state == "STOP_HOVER":

            # 保持当前位置
            self.target = [
                self.current[0],
                self.current[1],
                self.current[2],
            ]

            return

        # =====================================================
        # FLY TO
        #
        # 第一版暂不假装支持。
        # =====================================================

        if self.state == "UNSUPPORTED":

            if self.current_command:

                action_id = (
                    self.current_command
                    .get("action_id")
                )

                self.publish_failed(
                    action_id,
                    "UNSUPPORTED_ACTION",
                )

                self.current_command = None

            self.state = "IDLE"

            return

        # =====================================================
        # ORBIT
        # =====================================================

        if self.state == "ORBIT":

            self.execute_orbit()

            return

        # =====================================================
        # LANDING
        # =====================================================

        if self.state == "LANDING":

            if not self.armed:

                action_id = (
                    self.current_command
                    .get("action_id")
                )

                self.get_logger().info(
                    "🛬 LAND COMPLETE"
                )

                self.publish_completed(
                    action_id
                )

                self.current_command = None

                self.state = "IDLE"

            return

    # =========================================================
    # Start current command
    # =========================================================

    def start_current_command(self):

        if self.current_command is None:
            return

        action = self.current_command.get(
            "action"
        )

        action_id = self.current_command.get(
            "action_id"
        )

        params = self.current_command.get(
            "params",
            {},
        )

        self.get_logger().info(
            f"▶️ 开始执行: "
            f"{self.current_command}"
        )

        self.publish_executing(
            action_id
        )

        # =====================================================
        # TAKEOFF
        # =====================================================

        if action == "takeoff":

            altitude = params.get(
                "alt",
                2.0,
            )

            try:

                altitude = float(
                    altitude
                )

            except (
                TypeError,
                ValueError,
            ):

                self.publish_failed(
                    action_id,
                    "INVALID_ALT",
                )

                self.current_command = None
                self.state = "IDLE"

                return

            if altitude <= 0:

                self.publish_failed(
                    action_id,
                    "ALT_MUST_BE_POSITIVE",
                )

                self.current_command = None
                self.state = "IDLE"

                return

            self.target = [
                self.current[0],
                self.current[1],
                -abs(altitude),
            ]

            self.offboard_counter = 0

            self.state = "PREPARE"

            self.get_logger().info(
                f"🛫 TAKEOFF target="
                f"{altitude:.2f}m"
            )

            return

        # =====================================================
        # HOVER
        # =====================================================

        if action == "hover":

            self.target = [
                self.current[0],
                self.current[1],
                self.current[2],
            ]

            self.state = "HOVER"

            return

        # =====================================================
        # LAND
        # =====================================================

        if action == "land":

            self.send_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

            self.state = "LANDING"

            self.get_logger().info(
                "🛬 LAND command sent"
            )

            return

        # =====================================================
        # ORBIT
        # =====================================================

        if action == "orbit":

            if not self.prepare_orbit(
                params
            ):

                self.publish_failed(
                    action_id,
                    "INVALID_ORBIT_PARAMS",
                )

                self.current_command = None
                self.state = "IDLE"

                return

            self.state = "ORBIT"

            return

        # =====================================================
        # STOP
        # =====================================================

        if action == "stop":

            self.target = [
                self.current[0],
                self.current[1],
                self.current[2],
            ]

            self.state = "STOP_HOVER"

            return

        # =====================================================
        # Other actions
        #
        # 暂时明确返回 FAILED，
        # 不再 unknown → completed。
        # =====================================================

        if action in {
            "fly_to",
            "track_vel",
            "rtl",
            "turn",
        }:

            self.get_logger().warn(
                f"⚠️ action not implemented yet: "
                f"{action}"
            )

            self.state = "UNSUPPORTED"

            return

        # =====================================================
        # Unknown
        # =====================================================

        self.publish_failed(
            action_id,
            "UNKNOWN_ACTION",
        )

        self.current_command = None
        self.state = "IDLE"

    # =========================================================
    # Orbit preparation
    # =========================================================

    def prepare_orbit(self, params):

        center = params.get(
            "center"
        )

        radius = params.get(
            "radius"
        )

        speed = params.get(
            "speed"
        )

        if not isinstance(
            center,
            list
        ):

            return False

        if len(center) != 3:

            return False

        try:

            center_lon = float(
                center[0]
            )

            center_lat = float(
                center[1]
            )

            center_alt = float(
                center[2]
            )

            radius = float(
                radius
            )

            speed = float(
                speed
            )

        except (
            TypeError,
            ValueError,
        ):

            return False

        if radius <= 0:
            return False

        if speed <= 0:
            return False

        if not self.global_valid:

            self.get_logger().error(
                "❌ Orbit requires valid "
                "global position."
            )

            return False

        # -----------------------------------------------------
        # 协议：
        #
        # center = [longitude, latitude, altitude]
        # -----------------------------------------------------

        self.orbit_center_lon = (
            center_lon
        )

        self.orbit_center_lat = (
            center_lat
        )

        self.orbit_center_alt = (
            center_alt
        )

        self.orbit_radius = radius

        self.orbit_speed = speed

        # -----------------------------------------------------
        # 初始角度
        #
        # 当前 UAV 在圆心坐标系中的局部位置。
        # -----------------------------------------------------

        north, east = (
            self.geo_to_local(
                self.latitude,
                self.longitude,
                self.orbit_center_lat,
                self.orbit_center_lon,
            )
        )

        self.orbit_angle = math.atan2(
            east,
            north,
        )

        self.orbit_initialized = True

        self.get_logger().info(
            f"⭕ ORBIT prepared: "
            f"center=({center_lon}, "
            f"{center_lat}, "
            f"{center_alt}), "
            f"radius={radius:.2f}m, "
            f"speed={speed:.2f}m/s"
        )

        return True

    # =========================================================
    # Execute orbit
    # =========================================================

    def execute_orbit(self):

        if not self.orbit_initialized:

            action_id = (
                self.current_command
                .get("action_id")
            )

            self.publish_failed(
                action_id,
                "ORBIT_NOT_INITIALIZED",
            )

            self.current_command = None
            self.state = "IDLE"

            return

        if not self.global_valid:

            return

        # -----------------------------------------------------
        # Angular velocity
        #
        # omega = v / r
        # -----------------------------------------------------

        omega = (
            self.orbit_speed
            /
            self.orbit_radius
        )

        dt = 0.05

        self.orbit_angle += (
            omega * dt
        )

        # -----------------------------------------------------
        # Circle in local NED
        #
        # North = radius * cos(theta)
        # East  = radius * sin(theta)
        # Down  = center altitude relation
        # -----------------------------------------------------

        north = (
            self.orbit_radius
            *
            math.cos(
                self.orbit_angle
            )
        )

        east = (
            self.orbit_radius
            *
            math.sin(
                self.orbit_angle
            )
        )

        # -----------------------------------------------------
        # Convert center-relative
        # local coordinate into global local position.
        #
        # First calculate the UAV's current local position
        # relative to the orbit center.
        # -----------------------------------------------------

        center_north, center_east = (
            self.geo_to_local(
                self.latitude,
                self.longitude,
                self.orbit_center_lat,
                self.orbit_center_lon,
            )
        )

        # -----------------------------------------------------
        # Approximate local NED target.
        #
        # Because the existing PX4 interface is local NED,
        # we use the current local position as the reference
        # and calculate the orbit-center offset.
        # -----------------------------------------------------

        target_north = (
            self.current[0]
            +
            north
            -
            center_north
        )

        target_east = (
            self.current[1]
            +
            east
            -
            center_east
        )

        # -----------------------------------------------------
        # Altitude
        #
        # WGS84 altitude -> local NED relationship is not
        # identical to simply assigning -center_alt.
        #
        # For this first implementation we hold the current
        # NED altitude.
        # -----------------------------------------------------

        target_down = self.current[2]

        self.target = [
            target_north,
            target_east,
            target_down,
        ]

    # =========================================================
    # Geo → local NED
    #
    # Small-distance approximation.
    #
    # Output:
    # north [m]
    # east  [m]
    # =========================================================

    @staticmethod
    def geo_to_local(
        latitude,
        longitude,
        ref_latitude,
        ref_longitude,
    ):

        earth_radius = 6378137.0

        d_lat = math.radians(
            latitude
            -
            ref_latitude
        )

        d_lon = math.radians(
            longitude
            -
            ref_longitude
        )

        mean_lat = math.radians(
            (
                latitude
                +
                ref_latitude
            )
            /
            2.0
        )

        north = (
            d_lat
            *
            earth_radius
        )

        east = (
            d_lon
            *
            earth_radius
            *
            math.cos(
                mean_lat
            )
        )

        return north, east

    # =========================================================
    # Cancel current command
    # =========================================================

    def cancel_current_command(
        self,
        publish_status=True,
    ):

        if self.current_command is None:

            self.state = "IDLE"

            return

        action_id = (
            self.current_command
            .get("action_id")
        )

        action = (
            self.current_command
            .get("action")
        )

        self.get_logger().warn(
            f"🛑 Cancelling action: "
            f"{action} [{action_id}]"
        )

        if publish_status:

            self.publish_cancelled(
                action_id
            )

        self.current_command = None

        self.orbit_initialized = False

        self.state = "IDLE"

    # =========================================================
    # Completed
    # =========================================================

    def publish_completed(
        self,
        action_id,
    ):

        self.publish_status(
            action_id,
            "COMPLETED",
        )

    # =========================================================
    # Executing
    # =========================================================

    def publish_executing(
        self,
        action_id,
    ):

        self.publish_status(
            action_id,
            "EXECUTING",
        )

    # =========================================================
    # Cancelled
    # =========================================================

    def publish_cancelled(
        self,
        action_id,
    ):

        self.publish_status(
            action_id,
            "CANCELLED",
        )

    # =========================================================
    # Failed
    # =========================================================

    def publish_failed(
        self,
        action_id,
        reason,
    ):

        self.publish_status(
            action_id,
            "FAILED",
            reason,
        )

    # =========================================================
    # Status publisher
    # =========================================================

    def publish_status(
        self,
        action_id,
        state,
        reason=None,
    ):

        data = {
            "action_id": action_id,
            "state": state,
        }

        if reason is not None:

            data["reason"] = reason

        msg = String()

        msg.data = json.dumps(
            data,
            ensure_ascii=False,
        )

        self.status_publisher.publish(
            msg
        )

        self.get_logger().info(
            f"📡 status: {msg.data}"
        )

    # =========================================================
    # Offboard heartbeat
    # =========================================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.now()

        msg.position = True

        msg.velocity = False

        msg.acceleration = False

        msg.attitude = False

        msg.body_rate = False

        msg.thrust_and_torque = False

        msg.direct_actuator = False

        self.offboard_pub.publish(
            msg
        )

        self.offboard_counter += 1

    # =========================================================
    # Trajectory setpoint
    # =========================================================

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.now()

        msg.position = [
            float(self.target[0]),
            float(self.target[1]),
            float(self.target[2]),
        ]

        msg.velocity = [
            0.0,
            0.0,
            0.0,
        ]

        msg.acceleration = [
            0.0,
            0.0,
            0.0,
        ]

        msg.yaw = 0.0

        msg.yawspeed = 0.0

        self.setpoint_pub.publish(
            msg
        )

    # =========================================================
    # Vehicle Command
    # =========================================================

    def send_command(
        self,
        command,
        p1=0.0,
        p2=0.0,
    ):

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

        self.command_pub.publish(
            msg
        )

        self.get_logger().info(
            f"📤 send command {command}"
        )

    # =========================================================
    # Vehicle Status
    # =========================================================

    def status_callback(self, msg):

        self.armed = (
            msg.arming_state
            ==
            VehicleStatus.ARMING_STATE_ARMED
        )

        self.nav_state = (
            msg.nav_state
        )

    # =========================================================
    # Local position
    # =========================================================

    def position_callback(self, msg):

        self.current = [
            float(msg.x),
            float(msg.y),
            float(msg.z),
        ]

    # =========================================================
    # Global position
    # =========================================================

    def global_position_callback(self, msg):

        # PX4 VehicleGlobalPosition:
        #
        # lat/lon are normally 1e-7 degree integers.

        self.latitude = (
            float(msg.lat)
            / 1e7
        )

        self.longitude = (
            float(msg.lon)
            / 1e7
        )

        self.altitude = float(
            msg.alt
        )

        # Basic validity check

        if (
            abs(self.latitude) <= 90.0
            and
            abs(self.longitude) <= 180.0
        ):

            self.global_valid = True

    # =========================================================
    # Time
    # =========================================================

    def now(self):

        return int(
            self.get_clock()
            .now()
            .nanoseconds
            /
            1000
        )


# =============================================================
# main
# =============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = CommunicationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()