#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    

    map_yaml_file_path = PathJoinSubstitution(
        [FindPackageShare('robot_omni'), 'maps', 'hospital_map_v4.yaml']
    )

    params_file_path = PathJoinSubstitution(
        [FindPackageShare('robot_omni'), 'config', 'params.yaml']
    )

    nav2_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('robot_omni'), 'launch']
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('robot_omni'), 'rviz', 'localization.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map_yaml_file', default_value=map_yaml_file_path),
        DeclareLaunchArgument('params_file', default_value=params_file_path),

        # 1. Bật Gazebo, Robot và Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/gazebo_control.launch.py']),
            launch_arguments={'use_sim_time': use_sim}.items(),
        ),

        # 2. THÊM MỚI: Bật Nav2 Localization của hệ thống (SỬA CHỮ robot_omni THÀNH nav2_bringup)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
            ),
            launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim,
            'params_file': params_file,
            'autostart': autostart
        }.items()
        ),

        # 3. Bật RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            condition=IfCondition(start_rviz)
        ),
    ])