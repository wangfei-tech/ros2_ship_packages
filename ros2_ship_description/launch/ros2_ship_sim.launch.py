from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ros_ship_description = get_package_share_directory('ros2_ship_description')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # RViz 可选项
    rviz_config = os.path.join(ros_ship_description, 'config', 'urdf.rviz')

    return LaunchDescription([
        # Robot Description 发布
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ', os.path.join(ros_ship_description, 'robots', 'ros_ship.urdf.xacro')
                ])
            }]
        ),

        # 启动 Gazebo 仿真环境
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(ros_ship_description, 'world', 'open_water.world')
            }.items(),
        ),

        # 可选：加载 RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
