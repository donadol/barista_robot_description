import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

def generate_launch_description():

    xacro_file = 'barista_robot_model.urdf.xacro'
    package_description = "barista_robot_description"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_barista_description = get_package_share_directory(package_description)
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

    # Declare launch arguments
    include_laser_arg = DeclareLaunchArgument(
        'include_laser',
        default_value='true',
        description='Whether to include the laser scanner in the robot model'
    )

    # Xacro path
    print("Fetching XACRO ==>")
    robot_desc_path = os.path.join(pkg_barista_description, "xacro", xacro_file)

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch'), '/gazebo.launch.py']),
        launch_arguments={"verbose": "false"}.items(),
    )

    # Define robot names
    robot_name_1 = "rick"
    robot_name_2 = "morty"

    # Robot State Publisher for Robot 1 (Rick)
    robot_state_publisher_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_1,
        emulate_tty=True,
        parameters=[{
            'frame_prefix': robot_name_1 + '/',
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', robot_desc_path, ' include_laser:=', LaunchConfiguration('include_laser'), ' robot_name:=', robot_name_1]),
                value_type=str
            )
        }],
        output="screen"
    )

    # Robot State Publisher for Robot 2 (Morty)
    robot_state_publisher_robot2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_2,
        emulate_tty=True,
        parameters=[{
            'frame_prefix': robot_name_2 + '/',
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', robot_desc_path, ' include_laser:=', LaunchConfiguration('include_laser'), ' robot_name:=', robot_name_2]),
                value_type=str
            )
        }],
        output="screen"
    )

    # Spawn Robot 1 (Rick) in Gazebo
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot1',
        output='screen',
        arguments=[
            '-entity', robot_name_1,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-topic', robot_name_1 + '/robot_description'
        ]
    )

    # Spawn Robot 2 (Morty) in Gazebo
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot2',
        output='screen',
        arguments=[
            '-entity', robot_name_2,
            '-x', '1.0',
            '-y', '1.0',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-topic', robot_name_2 + '/robot_description'
        ]
    )

    # Static transform from world to rick/odom
    static_tf_world_to_rick = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_'+ robot_name_1,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot_name_1+'/odom']
    )

    # Static transform from world to morty/odom
    static_tf_world_to_morty = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_'+ robot_name_2,
        output='screen',
        arguments=['1.0', '1.0', '0', '0', '0', '0', 'world', robot_name_2+'/odom']
    )

    # RViz configuration
    rviz_config_dir = os.path.join(pkg_barista_description, 'rviz', 'two_robots.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Delay spawn and RViz to give robot_state_publisher time to start
    delayed_spawn_robot1 = TimerAction(
        period=2.0,
        actions=[spawn_robot1]
    )

    delayed_spawn_robot2 = TimerAction(
        period=2.5,
        actions=[spawn_robot2]
    )

    delayed_rviz = TimerAction(
        period=4.0,
        actions=[rviz_node]
    )

    # Create and return launch description
    return LaunchDescription([
        include_laser_arg,
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_barista_description, 'worlds', 'barista_empty.world'), ''],
            description='SDF world file'
        ),
        gazebo,
        robot_state_publisher_robot1,
        robot_state_publisher_robot2,
        static_tf_world_to_rick,
        static_tf_world_to_morty,
        delayed_spawn_robot1,
        delayed_spawn_robot2,
        delayed_rviz
    ])
