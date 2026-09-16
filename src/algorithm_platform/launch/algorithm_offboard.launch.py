
#!/usr/bin/env python3
"""
Gazebo 世界管理器 - 后台守护节点
- 不切换 Python 解释器，避免破坏终端状态
- 通过 sys.path 使用 venv 中的包
- 维护唯一的 Gazebo + PX4 仿真世界
- 提供 5 种运行状态
用法: ./gazebo_manager.py
"""

import os
import sys

# ============================================================
# 第一步：将 venv 的 site-packages 添加到 sys.path
# 这样系统 Python 也能使用 venv 中的包
# ============================================================
VENV_PATH = os.path.expanduser("~/algorithm_platform_ws/algwsvenv")
VENV_SITE_PACKAGES = os.path.join(
    VENV_PATH, 
    "lib", 
    f"python{sys.version_info.major}.{sys.version_info.minor}", 
    "site-packages"
)

# 如果 venv 存在，将其加入 Python 模块搜索路径
if os.path.exists(VENV_SITE_PACKAGES):
    if VENV_SITE_PACKAGES not in sys.path:
        sys.path.insert(0, VENV_SITE_PACKAGES)
        print(f"✅ 使用虚拟环境包: {VENV_SITE_PACKAGES}")
else:
    print(f"⚠️ 虚拟环境 site-packages 不存在: {VENV_SITE_PACKAGES}")
    print("   请先创建并安装依赖:")
    print("   cd ~/algorithm_platform_ws")
    print("   python3 -m venv algwsvenv")
    print("   source algwsvenv/bin/activate")
    print("   pip install rclpy")
# ============================================================

# ============================================================
# 现在导入依赖（从 venv 中加载）
# ============================================================
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String, Bool
    print("✅ rclpy 导入成功")
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    print("   请确保虚拟环境已安装依赖:")
    print("   source ~/algorithm_platform_ws/algwsvenv/bin/activate")
    print("   pip install rclpy")
    sys.exit(1)

import subprocess
import time
import signal
import threading
import json
from enum import Enum

# ============================================================
# 状态定义
# ============================================================
class GazeboState(Enum):
    STATE_WAIT = "WAIT"
    STATE_INIT = "INIT"
    STATE_RUNNING_NORMAL = "NORMAL"
    STATE_RUNNING_DEGRADED = "DEGRADED"
    STATE_WRONG = "WRONG"

    def __str__(self):
        return self.value


# ============================================================
# 配置
# ============================================================
HOME = os.path.expanduser("~")
PX4_ROOT = os.path.join(HOME, "px4/px4v1.15.2")
PX4_BIN = os.path.join(PX4_ROOT, "build/px4_sitl_default/bin/px4")

DRONE_CONFIG = {
    "model": "gz_x500_depth",
    "pose": (0.0, 0.0, 0.0),
    "instance_id": 1
}

CAMERA_CONFIG = {
    "width": 640,
    "height": 480,
    "fps": 15,
    "bitrate": 1000,
}


# ============================================================
# GazeboManager 节点
# ============================================================
class GazeboManager(Node):
    def __init__(self):
        super().__init__('gazebo_manager')
        
        self.state = GazeboState.STATE_WAIT
        self.state_lock = threading.Lock()
        self.gazebo_running = False
        self.gazebo_process = None
        self.startup_check_timer = None
        self._startup_attempts = 0
        
        self.status_pub = self.create_publisher(String, '/gazebo_manager/status', 10)
        self.ready_pub = self.create_publisher(Bool, '/gazebo_manager/ready', 10)
        
        self.create_subscription(String, '/gazebo_manager/command', self.command_callback, 10)
        
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.health_timer = self.create_timer(1.0, self.health_check)
        
        self.get_logger().info("🚀 Gazebo Manager 节点已启动")
        self.get_logger().info(f"📍 初始状态: {self.state}")
        
        self.start_gazebo()

    def set_state(self, new_state: GazeboState, reason: str = ""):
        with self.state_lock:
            old_state = self.state
            if old_state != new_state:
                self.state = new_state
                msg = f"🔄 状态切换: {old_state} → {new_state}"
                if reason:
                    msg += f" ({reason})"
                self.get_logger().info(msg)
                self.publish_status()
                
                if new_state == GazeboState.STATE_WRONG:
                    self.get_logger().error("❌ 进入故障锁定状态！")
                elif new_state == GazeboState.STATE_RUNNING_NORMAL:
                    self.get_logger().info("✅ 进入正常运行状态")
                    self.ready_pub.publish(Bool(data=True))

    def get_state(self) -> GazeboState:
        with self.state_lock:
            return self.state

    def is_ready(self) -> bool:
        state = self.get_state()
        return state in [GazeboState.STATE_RUNNING_NORMAL, GazeboState.STATE_RUNNING_DEGRADED]

    def command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            action = cmd.get('action', '').upper()
            self.get_logger().info(f"📨 收到命令: {action}")
            
            if action == 'START':
                if self.gazebo_running:
                    self.set_state(GazeboState.STATE_RUNNING_NORMAL, "任务启动")
                else:
                    self.get_logger().warning("⚠️ Gazebo 未运行")
            elif action == 'STOP':
                self.set_state(GazeboState.STATE_INIT, "任务停止")
            elif action == 'PAUSE':
                self.set_state(GazeboState.STATE_RUNNING_DEGRADED, "用户暂停")
            elif action == 'RESUME':
                if self.gazebo_running:
                    self.set_state(GazeboState.STATE_RUNNING_NORMAL, "用户恢复")
            elif action == 'RESET':
                self.get_logger().info("🔄 重置 Gazebo...")
                self.restart_gazebo()
            elif action == 'SHUTDOWN':
                self.get_logger().info("🛑 关闭 Gazebo...")
                self.shutdown_gazebo()
                self.set_state(GazeboState.STATE_WAIT, "用户关闭")
            else:
                self.get_logger().warning(f"⚠️ 未知命令: {action}")
        except Exception as e:
            self.get_logger().error(f"❌ 命令处理错误: {e}")

    def start_gazebo(self):
        if not os.path.exists(PX4_BIN):
            self.get_logger().error(f"❌ 找不到 PX4: {PX4_BIN}")
            self.set_state(GazeboState.STATE_WRONG, "PX4 不存在")
            return False

        if self.gazebo_running:
            self.get_logger().warning("⚠️ Gazebo 已在运行")
            return True

        self.get_logger().info("🚀 启动 Gazebo + PX4...")
        
        try:
            self._kill_all_gz_processes()
            
            env = self._get_env()
            env["PX4_GZ_MODEL_POSE"] = f"{DRONE_CONFIG['pose'][0]},{DRONE_CONFIG['pose'][1]},{DRONE_CONFIG['pose'][2]}"
            # env["GZ_SIM_RESOURCE_PATH"] = PX4_ROOT
            env["GZ_SIM_RESOURCE_PATH"] = os.path.join(PX4_ROOT,"Tools/simulation/gz/models")

            
            self.gazebo_process = subprocess.Popen(
                ["make", "px4_sitl", DRONE_CONFIG["model"]],
                cwd=PX4_ROOT,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            self.set_state(GazeboState.STATE_INIT, "等待 Gazebo 启动")
            self._startup_attempts = 0
            self.startup_check_timer = self.create_timer(1.0, self._check_gazebo_startup)
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 启动失败: {e}")
            self.set_state(GazeboState.STATE_WRONG, f"启动失败: {e}")
            return False

    def _check_gazebo_startup(self):
        if self._check_gz_running():
            self.gazebo_running = True
            self.set_state(GazeboState.STATE_INIT, "Gazebo 就绪")
            self.ready_pub.publish(Bool(data=False))
            self.get_logger().info("✅ Gazebo 已就绪")
            if self.startup_check_timer:
                self.startup_check_timer.cancel()
                self.startup_check_timer = None
        else:
            self._startup_attempts += 1
            if self._startup_attempts > 30:
                self.get_logger().error("❌ Gazebo 启动超时")
                self.set_state(GazeboState.STATE_WRONG, "启动超时")
                if self.startup_check_timer:
                    self.startup_check_timer.cancel()
                    self.startup_check_timer = None

    def restart_gazebo(self):
        self.shutdown_gazebo()
        time.sleep(2)
        self.start_gazebo()

    def shutdown_gazebo(self):
        if self.gazebo_process:
            try:
                self.gazebo_process.terminate()
            except:
                pass
            self.gazebo_process = None
        self.gazebo_running = False
        self._kill_all_gz_processes()
        self.get_logger().info("🛑 Gazebo 已关闭")

    def _get_env(self):
        env = os.environ.copy()
        env["GZ_CAMERA_WIDTH"] = str(CAMERA_CONFIG["width"])
        env["GZ_CAMERA_HEIGHT"] = str(CAMERA_CONFIG["height"])
        env["GZ_CAMERA_FPS"] = str(CAMERA_CONFIG["fps"])
        env["GZ_CAMERA_BITRATE"] = str(CAMERA_CONFIG["bitrate"])
        env["ROS2_IMAGE_COMPRESSION"] = "1"
        env["ROS2_IMAGE_COMPRESSION_MODE"] = "JPEG"
        env["ROS2_IMAGE_QUALITY"] = "50"
        return env

    def _check_gz_running(self):
        result = subprocess.run(["pgrep", "-f", "gz sim"], capture_output=True)
        return len(result.stdout) > 0

    def _kill_all_gz_processes(self):
        subprocess.run(["pkill", "-f", "px4"], capture_output=True)
        subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)
        subprocess.run(["pkill", "-f", "gzserver"], capture_output=True)
        subprocess.run(["pkill", "-f", "gzclient"], capture_output=True)
        time.sleep(1)

    def publish_status(self):
        self.get_logger().info("====publish_status被调用====")
        status = {
            'state': str(self.get_state()),
            'state_code': self.get_state().value,
            'gazebo_running': self.gazebo_running,
            'drone_model': DRONE_CONFIG['model'],
            'drone_pose': list(DRONE_CONFIG['pose']),
            'is_ready': self.is_ready(),
            'timestamp': time.time()
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def health_check(self):
        state = self.get_state()
        if state == GazeboState.STATE_WRONG:
            return
        if self.gazebo_running and not self._check_gz_running():
            self.get_logger().error("❌ Gazebo 意外退出！")
            self.set_state(GazeboState.STATE_WRONG, "Gazebo 进程丢失")
            self.gazebo_running = False


def main(args=None):
    rclpy.init(args=args)
    node = GazeboManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 收到中断信号")
    finally:
        node.shutdown_gazebo()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
