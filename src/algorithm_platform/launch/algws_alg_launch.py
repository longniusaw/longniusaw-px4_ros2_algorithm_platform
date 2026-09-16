#!/usr/bin/env python3
# 启动管家节点，管理其他节点，管家节点通过lifecyclenode机制来管理其他节点
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 管家节点
        Node(
            package='algorithm_platform',
            executable='algws_manager_node.py',
            name='algws_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}]
        ),
        # 如果以后有其他算法节点，加在这里
        # Node(...),
    ])
