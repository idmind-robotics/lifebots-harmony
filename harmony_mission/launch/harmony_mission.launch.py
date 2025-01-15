#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()
    
    pkg_share = get_package_share_directory("harmony_mission")
    # Mission Handler parameters
    def_missions_params = os.path.join(pkg_share, 'config', 'harmony_mission_params.yaml')
    missions_params = DeclareLaunchArgument(name="mission_params", default_value=def_missions_params, description="Mission handler parameters file")
    ld.add_action(missions_params)
    
    mission_controller_node = Node(
        package='harmony_mission',
        executable='harmony_mission_node',
        output='screen',
        emulate_tty=True,
        # name='harmony_mission_node',
        parameters=[{"control_freq": 20}, LaunchConfiguration('mission_params')])
    
    mission_controller_ns = GroupAction(actions=[
        PushRosNamespace("harmony"), 
        LogInfo(msg=["Starting Harmony Mission Controller."]),
        mission_controller_node,
        ],
    
    )
    ld.add_action(mission_controller_ns)

    return ld
