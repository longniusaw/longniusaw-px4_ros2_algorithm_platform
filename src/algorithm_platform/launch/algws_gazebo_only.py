#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    env = os.environ.copy()
    px4_path = "/home/longnius/px4/px4v1.15.2"
    
    env['GZ_SIM_RESOURCE_PATH'] = f"{px4_path}/build/px4_sitl_default/share/gz/models:{px4_path}/build/px4_sitl_default/share/gz/worlds"
    env['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    env['QT_QPA_PLATFORM'] = 'xcb'
    env['PX4_SIM_MODEL'] = 'gz_x500_depth'
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'gz_x500_depth'],
            cwd=px4_path,
            env=env,
            output='screen',
            shell=True,
            emulate_tty=True,
        ),
    ])