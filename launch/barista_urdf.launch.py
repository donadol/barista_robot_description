#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'barista_robot_model.urdf'
    package_description = "barista_robot_description"

    ####### DATA INPUT END ##########

    # Gazebo and package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_barista_description = get_package_share_directory(package_description)

    # Get install directory for Gazebo model path
    install_dir = get_package_prefix(package_description)

    # Set Gazebo model and plugin paths
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share'

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # URDF path
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(pkg_barista_description, "urdf", urdf_file)

    # RViz configuration
    rviz_config_dir = os.path.join(pkg_barista_description, 'rviz', 'urdf_vis.rviz')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='barista_robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_barista_robot',
        output='screen',
        arguments=[
            '-entity', 'barista_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-topic', '/robot_description'
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Create and return launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(get_package_share_directory('my_box_bot_gazebo'), 'worlds', 'box_bot_empty.world'), ''],
            description='SDF world file'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        rviz_node
    ])