#!/usr/bin/env python3

import json
from collections import deque

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)

from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition
)


class CommunicationNode(Node):

    def __init__(self):

        super().__init__('algws_communication')

        # =========================================================
        # PX4 QoS
        # =========================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # =========================================================
        # ROS2 command input
        # =========================================================

        self.create_subscription(
            String,
            '/algorithm/command/arbitrated',
            self.command_callback,
            10
        )

        # =========================================================
        # PX4 publishers
        # =========================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # =========================================================
        # PX4 subscribers
        # =========================================================

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            px4_qos
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        # =========================================================
        # State
        # =========================================================

        self.armed = False

        self.nav_state = (
            VehicleStatus.NAVIGATION_STATE_MAX
        )

        self.current = [
            0.0,
            0.0,
            0.0
        ]

        self.target = [
            0.0,
            0.0,
            0.0
        ]

        # =========================================================
        # Command queue
        # =========================================================

        self.command_queue = deque()

        self.current_command = None

        # =========================================================
        # State machine
        # =========================================================

        self.state = "IDLE"

        # =========================================================
        # Offboard preparation
        # =========================================================

        self.offboard_counter = 0

        # =========================================================
        # Parameters
        # =========================================================

        self.takeoff_tolerance = 0.1

        self.position_tolerance = 0.2

        # =========================================================
        # Timers
        # =========================================================

        # 20Hz Offboard heartbeat

        self.create_timer(
            0.05,
            self.publish_offboard_mode
        )

        # 50Hz trajectory setpoint

        self.create_timer(
            0.02,
            self.publish_setpoint
        )

        # 20Hz state machine

        self.create_timer(
            0.05,
            self.state_machine
        )

        # =========================================================
        # Startup
        # =========================================================

        self.get_logger().info(
            '🚀 Communication node started'
        )

    # =============================================================
    # Receive command
    # =============================================================

    def command_callback(self, msg):

        try:

            data = json.loads(msg.data)

        except json.JSONDecodeError:

            self.get_logger().error(
                f'❌ JSON解析失败: {msg.data}'
            )

            return

        action = data.get('action')

        if action is None:

            self.get_logger().warn(
                '⚠️ 指令没有 action'
            )

            return

        # =========================================================
        # Emergency stop
        # =========================================================

        if action == 'emergency_stop':

            self.command_queue.clear()

            self.current_command = None

            self.state = 'ERROR'

            self.get_logger().error(
                '🚨 EMERGENCY STOP'
            )

            return

        # =========================================================
        # Land
        # =========================================================

        if action == 'land':

            self.command_queue.append(data)

            self.get_logger().info(
                '📥 LAND command queued'
            )

            return

        # =========================================================
        # Normal command
        # =========================================================

        self.command_queue.append(data)

        self.get_logger().info(
            f'📥 command queued: {data}'
        )

        self.get_logger().info(
            f'📦 queue size: {len(self.command_queue)}'
        )

    # =============================================================
    # State machine
    # =============================================================

    def state_machine(self):

        # =========================================================
        # IDLE
        # =========================================================

        if self.state == 'IDLE':

            if self.current_command is None:

                if len(self.command_queue) == 0:

                    return

                self.current_command = (
                    self.command_queue.popleft()
                )

                self.get_logger().info(
                    f'▶️ 开始执行: {self.current_command}'
                )

            action = self.current_command.get(
                'action'
            )

            # -----------------------------------------------------
            # TAKEOFF
            # -----------------------------------------------------

            if action == 'takeoff':

                altitude = self.current_command.get(
                    'altitude',
                    2.0
                )

                self.target = [
                    self.current[0],
                    self.current[1],
                    -abs(altitude)
                ]

                self.offboard_counter = 0

                self.state = 'PREPARE'

                self.get_logger().info(
                    f'🛫 TAKEOFF target={altitude:.2f}m'
                )

            # -----------------------------------------------------
            # FLY RELATIVE
            # -----------------------------------------------------

            elif action == 'fly_relative':

                dx = self.current_command.get(
                    'dx',
                    0.0
                )

                dy = self.current_command.get(
                    'dy',
                    0.0
                )

                dz = self.current_command.get(
                    'dz',
                    0.0
                )

                self.target = [
                    self.target[0] + dx,
                    self.target[1] + dy,
                    self.target[2] - dz
                ]

                self.state = 'FLY_RELATIVE'

                self.get_logger().info(
                    f'🚁 FLY target='
                    f'({self.target[0]:.2f}, '
                    f'{self.target[1]:.2f}, '
                    f'{self.target[2]:.2f})'
                )

            # -----------------------------------------------------
            # HOVER
            # -----------------------------------------------------

            elif action == 'hover':

                self.target = [
                    self.current[0],
                    self.current[1],
                    self.current[2]
                ]

                self.state = 'HOVER'

                self.get_logger().info(
                    '🟢 HOVER'
                )

            # -----------------------------------------------------
            # LAND
            # -----------------------------------------------------

            elif action == 'land':

                self.send_command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND
                )

                self.state = 'LANDING'

                self.get_logger().info(
                    '🛬 LAND command sent'
                )

            else:

                self.get_logger().warn(
                    f'⚠️ unknown action: {action}'
                )

                self.finish_current_command()

        # =========================================================
        # PREPARE
        # =========================================================

        elif self.state == 'PREPARE':

            if self.offboard_counter >= 40:

                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0
                )

                self.get_logger().info(
                    '🔄 switch OFFBOARD'
                )

                self.state = 'WAIT_OFFBOARD'

        # =========================================================
        # WAIT OFFBOARD
        # =========================================================

        elif self.state == 'WAIT_OFFBOARD':

            if self.nav_state == (
                VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):

                self.get_logger().info(
                    '✅ OFFBOARD OK'
                )

                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0
                )

                self.state = 'WAIT_ARM'

        # =========================================================
        # WAIT ARM
        # =========================================================

        elif self.state == 'WAIT_ARM':

            if self.armed:

                self.get_logger().info(
                    '✅ ARM OK'
                )

                self.state = 'TAKEOFF'

        # =========================================================
        # TAKEOFF
        # =========================================================

        elif self.state == 'TAKEOFF':

            altitude = -self.current[2]

            target_altitude = -self.target[2]

            error = abs(
                altitude - target_altitude
            )

            if error <= self.takeoff_tolerance:

                self.get_logger().info(
                    f'🛫 TAKEOFF COMPLETE '
                    f'height={altitude:.2f}m'
                )

                self.state = 'HOVER'

                self.finish_current_command()

        # =========================================================
        # HOVER
        # =========================================================

        elif self.state == 'HOVER':

            # 如果当前命令就是hover
            if (
                self.current_command is not None
                and self.current_command.get('action')
                == 'hover'
            ):

                self.get_logger().info(
                    '🟢 HOVER COMPLETE'
                )

                self.finish_current_command()

            # TAKEOFF完成后进入这里
            else:

                self.get_logger().debug(
                    '🟢 hovering'
                )

                # TAKEOFF已经完成
                if self.current_command is None:

                    self.state = 'IDLE'

        # =========================================================
        # FLY RELATIVE
        # =========================================================

        elif self.state == 'FLY_RELATIVE':

            dx = (
                self.current[0]
                - self.target[0]
            )

            dy = (
                self.current[1]
                - self.target[1]
            )

            dz = (
                self.current[2]
                - self.target[2]
            )

            distance = (
                dx * dx +
                dy * dy +
                dz * dz
            ) ** 0.5

            if distance <= self.position_tolerance:

                self.get_logger().info(
                    f'🚁 FLY COMPLETE '
                    f'position='
                    f'({self.current[0]:.2f}, '
                    f'{self.current[1]:.2f}, '
                    f'{self.current[2]:.2f})'
                )

                self.finish_current_command()

        # =========================================================
        # LANDING
        # =========================================================

        elif self.state == 'LANDING':

            if not self.armed:

                self.get_logger().info(
                    '🛬 LAND COMPLETE'
                )

                self.finish_current_command()

        # =========================================================
        # ERROR
        # =========================================================

        elif self.state == 'ERROR':

            self.get_logger().error(
                '❌ Communication node ERROR state'
            )

    # =============================================================
    # Finish command
    # =============================================================

    def finish_current_command(self):

        if self.current_command is not None:

            self.get_logger().info(
                f'✅ command complete: '
                f'{self.current_command}'
            )

        self.current_command = None

        self.state = 'IDLE'

    # =============================================================
    # Offboard heartbeat
    # =============================================================

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

        self.offboard_pub.publish(msg)

        self.offboard_counter += 1

    # =============================================================
    # Trajectory setpoint
    # =============================================================

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.now()

        msg.position = [
            float(self.target[0]),
            float(self.target[1]),
            float(self.target[2])
        ]

        msg.velocity = [
            0.0,
            0.0,
            0.0
        ]

        msg.acceleration = [
            0.0,
            0.0,
            0.0
        ]

        msg.yaw = 0.0

        msg.yawspeed = 0.0

        self.setpoint_pub.publish(msg)

    # =============================================================
    # Vehicle Command
    # =============================================================

    def send_command(
        self,
        command,
        p1=0.0,
        p2=0.0
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

        self.command_pub.publish(msg)

        self.get_logger().info(
            f'📤 send command {command}'
        )

    # =============================================================
    # Vehicle status
    # =============================================================

    def status_callback(self, msg):

        self.armed = (
            msg.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

        self.nav_state = msg.nav_state

    # =============================================================
    # Position
    # =============================================================

    def position_callback(self, msg):

        self.current = [
            float(msg.x),
            float(msg.y),
            float(msg.z)
        ]

    # =============================================================
    # Timestamp
    # =============================================================

    def now(self):

        return int(
            self.get_clock()
            .now()
            .nanoseconds / 1000
        )


def main(args=None):

    rclpy.init(args=args)

    node = CommunicationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()