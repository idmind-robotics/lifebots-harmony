#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.event_handlers import (OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown)

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Start LaunchDescription
    ld = LaunchDescription()
    azure_pkg_share = get_package_share_directory("azure_kinect_ros_driver")

    #######################
    #     PARAMETERS      #
    #######################    
    ld.add_action(DeclareLaunchArgument('depth_enabled',              default_value="true",           description="Enable or disable the depth camera"))
    ld.add_action(DeclareLaunchArgument('depth_mode',                 default_value="WFOV_UNBINNED",  description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR"))
    ld.add_action(DeclareLaunchArgument('color_enabled',              default_value="true",           description="Enable or disable the color camera"))
    ld.add_action(DeclareLaunchArgument('color_format',               default_value="bgra",           description="The format of RGB camera. Valid options: bgra, jpeg"))
    ld.add_action(DeclareLaunchArgument('color_resolution',           default_value="1536P",          description="Resolution at which to run the color camera. Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P"))
    ld.add_action(DeclareLaunchArgument('fps',                        default_value="5",              description="FPS to run both cameras at. Valid options are 5, 15, and 30"))
    ld.add_action(DeclareLaunchArgument('point_cloud',                default_value="true",           description="Generate a point cloud from depth data. Requires depth_enabled"))
    ld.add_action(DeclareLaunchArgument('rgb_point_cloud',            default_value="true",           description="Colorize the point cloud using the RBG camera. Requires color_enabled and depth_enabled"))
    ld.add_action(DeclareLaunchArgument('point_cloud_in_depth_frame', default_value="false",          description="Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false)."))
    ld.add_action(DeclareLaunchArgument('body_tracking_enabled',      default_value="false",          description="If set to true the joint positions will be published as marker arrays"))

    azure_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(azure_pkg_share, 'launch', 'driver.launch.py')]),
        launch_arguments={
            'depth_enabled': LaunchConfiguration('depth_enabled'),
            'depth_mode': LaunchConfiguration('depth_mode'),
            'color_enabled': LaunchConfiguration('color_enabled'),
            'color_format': LaunchConfiguration('color_format'),
            'color_resolution': LaunchConfiguration('color_resolution'),
            'fps': LaunchConfiguration('fps'),
            'point_cloud': LaunchConfiguration('point_cloud'),
            'rgb_point_cloud': LaunchConfiguration('rgb_point_cloud'),
            'point_cloud_in_depth_frame': LaunchConfiguration('point_cloud_in_depth_frame'),
            'body_tracking_enabled': LaunchConfiguration('body_tracking_enabled'),
            }.items(),
    )
    ld.add_action(azure_launch)

    return ld