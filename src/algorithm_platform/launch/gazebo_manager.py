import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import threading
import json


class GazeboManager(Node):
    def __init__(self):
        super().__init__("gazebo_manager")

        # =========【原有5个状态，保持不变】=========
        self.STATE_WAIT = "WAIT"
        self.STATE_INIT = "INIT"
        self.STATE_NORMAL = "NORMAL"
        self.STATE_PAUSE = "PAUSE"
        self.STATE_ERROR = "ERROR"
        self.state = self.STATE_WAIT
        self.state_lock = threading.Lock()

        self.gazebo_process = None

        # =========【新增：INIT阶段桥接与计时变量】=========
        self.init_enter_time = None
        self.bridge_proc = None
        self.bridge_started = False

        # ROS发布/订阅
        self.status_pub = self.create_publisher(String, "/gazebo_manager/status", 10)
        self.cmd_sub = self.create_subscription(
            String, "/gazebo_manager/command", self.command_callback, 10
        )

        # 定时器：状态上报0.5s，健康巡检1s
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.health_timer = self.create_timer(1.0, self.health_check)

        # 延时启动仿真（避免__init__阻塞ROS事件循环）
        self.delay_start_timer = self.create_timer(1.5, self.delayed_start_gazebo)
        self.get_logger().info("🚀 GazeboManager 节点初始化完成")

    def set_state(self, new_state):
        with self.state_lock:
            if self.state == new_state:
                return
            old = self.state
            self.state = new_state
            self.get_logger().info(f"🔄 状态切换: {old} → {new_state}")
            #切入INIT时重置计时与bridge标记
            if new_state == self.STATE_INIT:
                self.init_enter_time = time.time()
                self.bridge_started = False

    def delayed_start_gazebo(self):
        self.delay_start_timer.cancel()
        self.get_logger().info("⏱延时结束，开始拉起Gazebo仿真")
        self.set_state(self.STATE_INIT)
        self.start_gazebo()

    def start_gazebo(self):
        """这里替换成你自己原来启动PX4+Gazebo的命令"""
        # =========请把下面命令改成你原本启动gazebo的代码=========
        PX4_ROOT = "/home/longnius/px4/px4v1.15.2"
        cmd = ["make", "-C", PX4_ROOT, "px4_sitl", "gz_x500_depth"]
        try:
            self.gazebo_process = subprocess.Popen(cmd)
            self.get_logger().info("✅Gazebo仿真进程已启动")
        except Exception as e:
            self.get_logger().error(f"❌启动Gazebo失败:{e}")
            self.set_state(self.STATE_ERROR)

    def health_check(self):
        with self.state_lock:
            cur_state = self.state

        if cur_state == self.STATE_INIT:
            now = time.time()
            elapsed = now - self.init_enter_time
            #前10s只等待，不启动bridge
            if elapsed < 20.0:
                self.get_logger().info(f"INIT等待中：{elapsed:.1f}/20s，尚未启动bridge")
                return
            #满10s启动bridge，仅一次
            if not self.bridge_started:
                self.get_logger().info("⏱INIT已满20s，启动ros_gz_bridge")
                bridge_cmd = [
                    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                    "/depth_camera@sensor_msgs/msg.Image[gz.msgs.Image",
                    "/camera@sensor_msgs/msg.Image[gz.msgs.Image"
                ]
                self.bridge_proc = subprocess.Popen(bridge_cmd)
                self.bridge_started = True
            #检测：任意一个摄像头话题存在即就绪
            ok = self._check_any_camera_topic_up()
            if ok:
                self.get_logger().info("✅bridge运行成功，/camera或/depth_camera已上线，INIT就绪，等待管家START指令")
            else:
                self.get_logger().info("⌛bridge已启动，等待/camera或/depth_camera出现……")

        # ======================在此追加你原有其他状态(WAIT/NORMAL/PAUSE/ERROR)的health_check逻辑=====================
        # if cur_state == self.STATE_WAIT:
        #    ...
        # elif cur_state == self.STATE_NORMAL:
        #    ...

    def _check_any_camera_topic_up(self):
        res = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True
        )
        topics = set(res.stdout.splitlines())
        return "/camera" in topics or "/depth_camera" in topics

    def publish_status(self):
        """定时向外广播当前状态，不修改状态机"""
        with self.state_lock:
            status_data = {
                "state": self.state,
                "timestamp": time.time()
            }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)

    def command_callback(self, msg):
        """接收管家下发的START指令，INIT→NORMAL（原生跳转逻辑）"""
        cmd = msg.data.strip()
        with self.state_lock:
            if cmd == "START" and self.state == self.STATE_INIT:
                self.set_state(self.STATE_NORMAL)
                self.get_logger().info("📥收到START指令，进入NORMAL正常运行")
            elif cmd == "PAUSE" and self.state == self.STATE_NORMAL:
                self.set_state(self.STATE_PAUSE)
            elif cmd == "RESUME" and self.state == self.STATE_PAUSE:
                self.set_state(self.STATE_NORMAL)

    def shutdown_gazebo(self):
        #关闭bridge进程
        if self.bridge_proc is not None:
            try:
                self.bridge_proc.terminate()
                self.bridge_proc.wait()
                self.get_logger().info("🛑ros_gz_bridge进程已销毁")
            except Exception as e:
                self.get_logger().warn(f"终止bridge异常:{e}")
        #关闭gazebo仿真进程
        if self.gazebo_process is not None:
            try:
                self.gazebo_process.terminate()
                self.gazebo_process.wait()
                self.get_logger().info("🛑Gazebo仿真进程已销毁")
            except Exception as e:
                self.get_logger().warn(f"终止Gazebo异常:{e}")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑捕获Ctrl+C，准备退出")
    finally:
        node.shutdown_gazebo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
