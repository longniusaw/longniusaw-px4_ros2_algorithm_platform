#!/usr/bin/env python3
"""
平台总管家
1、启动 gazebo_manager
2、等待INIT →下发START →等待NORMAL
3、进入NORMAL后，开始采集并展示全部算法节点状态
4、后台持续监控节点健康，Ctrl+C全部清理
"""
import os
import sys
import subprocess
import time
import signal
import threading
import json
from typing import List, Dict

# =====================配置区，按需修改=====================
ROS2_SETUP = "/opt/ros/jazzy/setup.bash"
VENV_PATH = os.path.expanduser("~/algorithm_platform_ws/algwsvenv")
VENV_PYTHON = os.path.join(VENV_PATH, "bin", "python3")
GAZEBO_MANAGER_SCRIPT = os.path.expanduser(
    "~/algorithm_platform_ws/src/algorithm_platform/launch/gazebo_manager.py"
)
# 算法节点清单
ALGORITHM_NODES = [
    "arbiter_node",
    "fault_monitor_node",
    "global_planner_node",
    "local_planner_node",
    "slam_interface_node",
    "px4_bridge_client"
]
GAZEBO_START_TIMEOUT = 30    # 等待INIT最大时长
WAIT_NORMAL_TIMEOUT = 20     # 等待NORMAL最大时长
NODE_MONITOR_INTERVAL = 2    # 节点巡检间隔
STATUS_DISPLAY_INTERVAL = 30 # 每隔30秒打印一次状态表
# ==========================================================

processes = []
stop_event = threading.Event()
gazebo_proc = None

def signal_handler(sig, frame):
    """Ctrl+C终止全部进程"""
    print("\n🛑 收到停止信号，正在清理所有进程...")
    stop_event.set()
    if gazebo_proc is not None:
        try:
            gazebo_proc.terminate()
        except Exception:
            pass
    # 清理残留仿真与ros进程
    subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)
    subprocess.run(["pkill", "-f", "px4"], capture_output=True)
    subprocess.run(["pkill", "-f", "gazebo_manager"], capture_output=True)
    subprocess.run(["pkill", "-f", "ros2"], capture_output=True)
    print("✅全部进程清理完成")
    sys.exit(0)

def get_gazebo_manager_state() -> str | None:
    """订阅/gazebo_manager/status获取state字段，返回INIT/NORMAL/WAIT等"""
    cmd = (
        f"source {ROS2_SETUP} && "
        f"ros2 topic echo /gazebo_manager/status  --timeout 3"
    )
    try:
        res = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=5
        )
        lines = res.stdout.strip().splitlines()
        for line in lines:
            try:
                data = json.loads(line)
                return data.get("state")
            except json.JSONDecodeError:
                continue
        return None
    except Exception:
        return None

def send_start_command():
    """向gazebo_manager发送START指令"""
    cmd = (
        f"source {ROS2_SETUP} && "
        f'ros2 topic pub /gazebo_manager/command std_msgs/msg/String "{{\"data\":\"{{\\\"action\\\":\\\"START\\\"}}\"}}" --once'
    )
    subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=5)

def get_all_ros_nodes() -> List[str]:
    """获取当前全部运行的ROS节点名称"""
    cmd = f"source {ROS2_SETUP} && ros2 node list"
    try:
        res = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=5
        )
        nodes_raw = [n.strip().lstrip("/") for n in res.stdout.splitlines() if n.strip()]
        return nodes_raw
    except Exception:
        return []

def refresh_node_status() -> Dict[str, str]:
    """对比名单，生成每个节点运行状态"""
    online_nodes = get_all_ros_nodes()
    status_map = {}
    for name in ALGORITHM_NODES:
        if name in online_nodes:
            status_map[name] = "✅运行中"
        else:
            status_map[name] = "❌未启动"
    return status_map

def print_node_table(status_map: Dict[str, str]):
    """打印节点状态清单"""
    total = len(ALGORITHM_NODES)
    running = sum(1 for s in status_map.values() if "✅" in s)
    print("\n" + "="*60)
    print(f"📊算法节点清单｜总数:{total}｜在线:{running}/{total}")
    print("="*60)
    for node_name, state in status_map.items():
        print(f"    {state}    {node_name}")
    if running < total:
        failed = [k for k,v in status_map.items() if "❌" in v]
        print(f"\n⚠️离线节点：{', '.join(failed)}")
    else:
        print("\n✅全部算法节点正常在线！")
    print("="*60)

def monitor_loop():
    """后台持续巡检节点，定时打印状态表"""
    last_print = time.time()
    while not stop_event.is_set():
        status = refresh_node_status()
        now = time.time()
        if now - last_print > STATUS_DISPLAY_INTERVAL:
            print_node_table(status)
            last_print = now
        time.sleep(NODE_MONITOR_INTERVAL)

def launch_gazebo_manager():
    """启动gazebo_manager，单行命令消除换行bug"""
    if not os.path.exists(GAZEBO_MANAGER_SCRIPT):
        print(f"❌找不到gazebo_manager脚本：{GAZEBO_MANAGER_SCRIPT}")
        sys.exit(1)
    print("="*60)
    print("🚀启动 Gazebo Manager")
    print("="*60)
    # 清除旧进程
    subprocess.run(["pkill", "-f", "gazebo_manager"],capture_output=True)
    time.sleep(1)
    cmd = f"source {ROS2_SETUP} && python3 {GAZEBO_MANAGER_SCRIPT}"

    proc = subprocess.Popen(["bash","-c",cmd])
    global gazebo_proc
    gazebo_proc = proc
    print(f"✅Gazebo Manager已后台启动，PID={proc.pid}")
    return proc

def main():
    signal.signal(signal.SIGINT,signal_handler)
    print("="*60)
    print("🏛️平台总管家启动")
    print("="*60)
    print(f"📋待管理算法节点数量：{len(ALGORITHM_NODES)}")
    print("="*60)

    #1、启动gazebo_manager
    launch_gazebo_manager()

    #2、等待进入INIT状态
    print("\n⏳等待 Gazebo Manager 进入 INIT 状态...")
    init_ok = False
    for t in range(GAZEBO_START_TIMEOUT):
        state = get_gazebo_manager_state()
        if state == "INIT":
            print("✅成功进入 INIT 状态！")
            init_ok = True
            break
        time.sleep(1)
        if (t+1)%10==0:
            print(f"   ...等待({t+1}s),当前状态:{state}")
    if not init_ok:
        print("⚠️等待INIT超时，退出")
        signal_handler(None,None)

    #3、下发START指令
    print("\n📨发送START命令，请求切换至NORMAL状态")
    send_start_command()

    #4、等待切换至NORMAL，只有到达NORMAL之后才开始监控节点名单
    print("⏳等待 Gazebo Manager 进入 NORMAL 状态...")
    normal_ok = False
    for t in range(WAIT_NORMAL_TIMEOUT):
        state = get_gazebo_manager_state()
        if state == "NORMAL":
            print("✅成功进入 NORMAL，开始监控算法节点！")
            normal_ok = True
            break
        time.sleep(1)
        if (t+1)%5==0:
            print(f"   ...等待({t+1}s),当前状态:{state}")
    if not normal_ok:
        print("⚠️未能进入NORMAL状态，依然启动节点监控")

    #5、首次立刻打印一份节点清单
    first_status = refresh_node_status()
    print_node_table(first_status)

    #6、开启后台线程持续监控
    monitor_thread = threading.Thread(target=monitor_loop,daemon=True)
    monitor_thread.start()

    #主循环常驻等待Ctrl+C
    print("\n💡平台监控已运行，按下 Ctrl+C 停止全部仿真与节点")
    while not stop_event.is_set():
        time.sleep(1)

if __name__ == "__main__":
    main()
