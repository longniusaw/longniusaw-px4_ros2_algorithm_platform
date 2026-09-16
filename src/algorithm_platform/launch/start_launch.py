#!/usr/bin/env python3
# algorithm_platform/launch/bringup.launch.py

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前包路径
    pkg_path = get_package_share_directory('algorithm_platform')
    
    # 获取当前环境变量（让子进程继承）
    env = os.environ.copy()
    
    # ==========================================
    # 1. 启动 PX4 SITL + Gazebo Harmonic 仿真
    # ==========================================
    # ⚠️ 把下面的路径改成你实际 PX4-Autopilot 的路径！
    px4_autopilot_path = "/home/longnius/px4/px4v1.15.2"  # 👈 这里要修改！
    
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth'],
        cwd=px4_autopilot_path,
        env=env,
        output='screen',
        shell=True,
    )
    
    # ==========================================
    # 2. 启动管家节点
    # ==========================================
    manager_node = Node(
        package='algorithm_platform',
        executable='manager_node.py',
        name='robot_manager',
        output='screen',
        emulate_tty=True,  # 让 Python 的 print 能及时输出
    )
    
    # ==========================================
    # 3. 启动通讯节点（如果你有的话）
    # ==========================================
    # comm_node = Node(
    #     package='algorithm_platform',
    #     executable='communication_node.py',
    #     name='communication_node',
    #     output='screen',
    #     emulate_tty=True,
    #     # parameters=[os.path.join(pkg_path, 'config', 'comm_params.yaml')],  # 如果有参数文件
    # )
    
    return LaunchDescription([
        px4_sitl,
        manager_node,
        # comm_node,  # 暂时注释掉也行，等写好了再打开
    ])