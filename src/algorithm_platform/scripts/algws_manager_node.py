#!/usr/bin/env python3
# scripts/algws_manager_node.py

"""
Algorithm Platform Manager

职责：
1. 管理 Algorithm Platform 内部节点的启动/停止
2. 维护节点状态
3. 检查节点进程是否存活
4. 对异常退出的节点进行简单恢复
5. 管理平台自身的 Lifecycle

注意：
Manager 不负责：
- UAV 任务逻辑
- Commanding
- Arbitration
- NAV
- SLAM
- PX4 控制
"""

import os
import subprocess

import rclpy
from rclpy.lifecycle import (
    LifecycleNode,
    State,
    TransitionCallbackReturn,
)
from rclpy.executors import MultiThreadedExecutor


class AlgorithmManager(LifecycleNode):

    def __init__(self):
        super().__init__('algws_manager')

        self.get_logger().info(
            '🏗️ Algorithm Platform Manager 正在初始化...'
        )

        # ============================================================
        # 节点注册表
        # ============================================================
        #
        # status:
        #   stopped  -> 没有运行
        #   running  -> 进程正在运行
        #   error    -> 进程异常退出
        #
        # lifecycle:
        #   False -> 普通 ROS 2 Node
        #   True  -> ROS 2 LifecycleNode
        #
        # auto_start:
        #   True  -> Manager ACTIVE 后自动启动
        #   False -> 不自动启动
        #
        self.node_registry = {

            # --------------------------------------------------------
            # Arbitration Node
            # --------------------------------------------------------
            'arbitration': {
                'package': 'algorithm_platform',
                'executable': 'algws_arbitration_node.py',

                'status': 'stopped',
                'pid': None,

                'lifecycle': False,
                'auto_start': True,

                # 是否允许异常退出后自动重启
                'auto_restart': True,
            },

            # --------------------------------------------------------
            # Communication Node
            # --------------------------------------------------------
            'communication': {
                'package': 'algorithm_platform',
                'executable': 'algws_communication_node.py',

                'status': 'stopped',
                'pid': None,

                'lifecycle': False,
                'auto_start': True,

                'auto_restart': True,
            },

            # --------------------------------------------------------
            # Command Test Node
            #
            # 这是测试 C / coworker 的模拟节点。
            #
            # 正式运行时不自动启动。
            # --------------------------------------------------------
            # 'command_test': {
            #     'package': 'algorithm_platform',
            #     'executable': 'algws_command_test',

            #     'status': 'stopped',
            #     'pid': None,

            #     'lifecycle': False,

            #     # 测试节点不应该随着正式系统自动启动
            #     'auto_start': False,

            #     'auto_restart': False,
            # },
            'commanding': {
                'package': 'algorithm_platform',
                'executable': 'algws_commandin_node.py',
                'status': 'stopped',
                'pid': None,
                'lifecycle': False,
                'auto_start': True,
                'auto_restart': True,
            },

            # --------------------------------------------------------
            # 以后可以添加：
            #
            # 'commanding': {...}
            # 'nav': {...}
            # 'slam': {...}
            #
        }

        # 保存启动后的 subprocess 对象
        #
        # 例如：
        #
        # self.processes['arbitration']
        #
        # 就是 ArbitrationNode 对应的 Linux process。
        #
        self.processes = {}

        # 保存环境变量
        #
        # 这样 subprocess 启动 ros2 run 时，
        # 可以继承当前 shell 的 ROS 2 环境。
        #
        self.env = os.environ.copy()

        # ------------------------------------------------------------
        # 定时打印状态
        # ------------------------------------------------------------
        self.status_timer = self.create_timer(
            10.0,
            self._print_registry_status
        )

        self.health_timer = None

    # =================================================================
    # Lifecycle: CONFIGURE
    # =================================================================

    def on_configure(
        self,
        state: State
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            '⚙️ Manager → CONFIGURE'
        )

        self.get_logger().info(
            '📋 正在检查节点配置...'
        )

        for name, info in self.node_registry.items():

            self.get_logger().info(
                f'   {name}: '
                f'package={info["package"]}, '
                f'executable={info["executable"]}, '
                f'auto_start={info["auto_start"]}'
            )

        return TransitionCallbackReturn.SUCCESS

    # =================================================================
    # Lifecycle: ACTIVATE
    # =================================================================

    def on_activate(
        self,
        state: State
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            '▶️ Manager → ACTIVE'
        )

        self.get_logger().info(
            '🚀 开始启动 auto_start 节点...'
        )

        # ------------------------------------------------------------
        # 启动需要自动启动的节点
        # ------------------------------------------------------------
        for name, info in self.node_registry.items():

            if info['auto_start']:
                self._start_node(name)

        # ------------------------------------------------------------
        # 创建健康检查定时器
        # ------------------------------------------------------------
        self.health_timer = self.create_timer(
            5.0,
            self._check_health
        )

        return TransitionCallbackReturn.SUCCESS

    # =================================================================
    # 启动节点
    # =================================================================

    def _start_node(self, name):
        """
        启动一个 ROS 2 Node。

        实际执行：

            ros2 run <package> <executable>

        例如：

            ros2 run algorithm_platform algws_arbitration_node.py
        """

        if name not in self.node_registry:
            self.get_logger().error(
                f'❌ 未知节点: {name}'
            )
            return False

        info = self.node_registry[name]

        # ------------------------------------------------------------
        # 防止重复启动
        # ------------------------------------------------------------
        if info['status'] == 'running':

            self.get_logger().warn(
                f'⚠️ {name} 已经在运行'
            )

            return False

        package = info['package']
        executable = info['executable']

        self.get_logger().info(
            f'🚀 启动节点: {name}'
        )

        self.get_logger().info(
            f'   package    = {package}'
        )

        self.get_logger().info(
            f'   executable = {executable}'
        )

        try:

            # --------------------------------------------------------
            # 启动 ROS 2 Node
            # --------------------------------------------------------
            #
            # subprocess.Popen()
            #
            # 会创建一个新的 Linux 进程。
            #
            # 不使用 wait()，因为我们不希望 Manager 被这个
            # 子进程阻塞。
            #
            proc = subprocess.Popen(
                [
                    'ros2',
                    'run',
                    package,
                    executable
                ],
                env=self.env,
                stdout=None,
                stderr=None,
                text=True
            )

            # --------------------------------------------------------
            # 保存进程信息
            # --------------------------------------------------------
            self.processes[name] = proc

            info['pid'] = proc.pid
            info['status'] = 'running'

            self.get_logger().info(
                f'✅ {name} 启动成功 '
                f'(PID: {proc.pid})'
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f'❌ 启动 {name} 失败: {e}'
            )

            info['status'] = 'error'
            info['pid'] = None

            return False

    # =================================================================
    # 健康检查
    # =================================================================

    def _check_health(self):
        """
        检查所有被 Manager 启动的进程。

        注意：
        这里检查的是 Linux process 是否还存在。

        它还不是完整的 ROS 2 Node health check。
        """

        for name, info in self.node_registry.items():

            if info['status'] != 'running':
                continue

            proc = self.processes.get(name)

            if proc is None:
                continue

            # --------------------------------------------------------
            # poll()
            #
            # None:
            #     进程还在运行
            #
            # 非 None:
            #     进程已经退出
            # --------------------------------------------------------
            return_code = proc.poll()

            if return_code is None:
                continue

            # --------------------------------------------------------
            # Node 已经退出
            # --------------------------------------------------------
            old_pid = info['pid']

            self.get_logger().warn(
                f'💥 {name} 已退出 '
                f'(PID: {old_pid}, '
                f'return code: {return_code})'
            )

            info['status'] = 'error'
            info['pid'] = None

            self.processes.pop(name, None)

            # --------------------------------------------------------
            # 自动恢复
            # --------------------------------------------------------
            if info.get('auto_restart', False):

                self.get_logger().warn(
                    f'🔄 {name} 开启了自动恢复，准备重新启动...'
                )

                self._start_node(name)

    # =================================================================
    # 打印状态
    # =================================================================

    def _print_registry_status(self):

        self.get_logger().info(
            '📋 ==============================='
        )

        self.get_logger().info(
            '📋 Algorithm Platform Node Status'
        )

        self.get_logger().info(
            '📋 ==============================='
        )

        for name, info in self.node_registry.items():

            status_icon = {

                'running': '🟢',
                'stopped': '⚪',
                'error': '🔴',

                'configuring': '🟡',
                'active': '🟢',
                'inactive': '🟡',

            }.get(
                info['status'],
                '❓'
            )

            pid = info['pid']

            if pid is None:
                pid_text = 'N/A'
            else:
                pid_text = str(pid)

            self.get_logger().info(
                f'  {status_icon} '
                f'{name}: '
                f'{info["status"]} '
                f'(PID: {pid_text})'
            )

    # =================================================================
    # 停止所有节点
    # =================================================================

    def _stop_all_nodes(self):

        self.get_logger().info(
            '🛑 正在停止所有由 Manager 启动的节点...'
        )

        for name, proc in list(self.processes.items()):

            info = self.node_registry[name]

            self.get_logger().info(
                f'   ✋ 停止 {name} '
                f'(PID: {proc.pid})'
            )

            try:

                # ----------------------------------------------------
                # terminate()
                #
                # 向进程发送终止信号。
                # 相比 kill() 更温和。
                # ----------------------------------------------------
                proc.terminate()

                # ----------------------------------------------------
                # 最多等待 3 秒
                # ----------------------------------------------------
                proc.wait(timeout=3)

                self.get_logger().info(
                    f'   ✅ {name} 已停止'
                )

            except subprocess.TimeoutExpired:

                self.get_logger().warn(
                    f'   ⚠️ {name} 3 秒内没有退出，强制 kill'
                )

                proc.kill()

                try:
                    proc.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass

            except Exception as e:

                self.get_logger().error(
                    f'   ❌ 停止 {name} 时发生错误: {e}'
                )

            # --------------------------------------------------------
            # 更新 registry
            # --------------------------------------------------------
            info['status'] = 'stopped'
            info['pid'] = None

        self.processes.clear()

    # =================================================================
    # Lifecycle: DEACTIVATE
    # =================================================================

    def on_deactivate(
        self,
        state: State
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            '⏸️ Manager → DEACTIVATE'
        )

        #
        # V1 暂时不停止子节点。
        #
        # 因为 DEACTIVATE 是 Manager 自己的 Lifecycle 状态，
        # 并不意味着所有算法节点必须立即退出。
        #
        # 后续如果需要，可以定义明确的策略。
        #

        return TransitionCallbackReturn.SUCCESS

    # =================================================================
    # Lifecycle: CLEANUP
    # =================================================================

    def on_cleanup(
        self,
        state: State
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            '🧹 Manager → CLEANUP'
        )

        # 停止所有子节点
        self._stop_all_nodes()

        return TransitionCallbackReturn.SUCCESS

    # =================================================================
    # Lifecycle: SHUTDOWN
    # =================================================================

    def on_shutdown(
        self,
        state: State
    ) -> TransitionCallbackReturn:

        self.get_logger().info(
            '🛑 Manager → SHUTDOWN'
        )

        # ------------------------------------------------------------
        # 为了安全，再检查一次子进程
        # ------------------------------------------------------------
        if self.processes:
            self._stop_all_nodes()

        return TransitionCallbackReturn.SUCCESS


# =====================================================================
# main
# =====================================================================

def main(args=None):

    rclpy.init(args=args)

    # ------------------------------------------------------------
    # MultiThreadedExecutor
    #
    # 允许 ROS 2 回调在多个线程中执行。
    #
    # 对现在这个 Manager 来说不是绝对必须，
    # 但以后增加多个 timer / service / topic callback 时比较方便。
    # ------------------------------------------------------------
    executor = MultiThreadedExecutor()

    manager = AlgorithmManager()

    executor.add_node(manager)

    try:

        executor.spin()

    except KeyboardInterrupt:

        manager.get_logger().info(
            '🛑 收到 Ctrl+C，正在关闭 Manager...'
        )

    finally:

        # ------------------------------------------------------------
        # 确保子进程退出
        # ------------------------------------------------------------
        manager._stop_all_nodes()

        executor.shutdown()

        manager.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()