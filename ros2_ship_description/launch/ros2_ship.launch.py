from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    description_pkg = get_package_share_directory('ros2_ship_description')
    control_pkg = get_package_share_directory('ros2_ship_control')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    # URDF file path
    xacro_file = os.path.join(description_pkg, 'robots', 'ros_ship.urdf.xacro')

    # RViz config
    rviz_config = os.path.join(description_pkg, 'config', 'urdf.rviz')

    # Buoy files
    red_buoy = os.path.join(description_pkg, 'world', 'buoy', 'urdf', 'red_buoy.urdf')
    green_buoy = os.path.join(description_pkg, 'world', 'buoy', 'urdf', 'green_buoy.urdf')

    # World file
    world_file = os.path.join(description_pkg, 'world', 'open_water.world')

    return LaunchDescription([
        # Robot Description using xacro
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command(['xacro ', xacro_file]),
            description='URDF of the robot'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # Spawn ship
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            arguments=['-topic', 'robot_description', '-entity', 'ros_ship'],
            output='screen'
        ),

        # Spawn red buoy
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='red_buoy_spawner',
            namespace='red_buoy_01',
            arguments=['-file', red_buoy, '-entity', 'red_buoy_01', '-x', '10', '-y', '0', '-z', '0'],
            output='screen'
        ),

        # Spawn green buoy
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='green_buoy_spawner',
            namespace='green_buoy_01',
            arguments=['-file', green_buoy, '-entity', 'green_buoy_01', '-x', '10', '-y', '3', '-z', '0'],
            output='screen'
        ),

        # Include control launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg, 'launch', 'ros2_ship_control.launch.py')
            )
        ),

        # Include Gazebo world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'paused': 'true',
                'use_sim_time': 'true',
                'world': world_file
            }.items()
        ),
    ])
