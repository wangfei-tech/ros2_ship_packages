from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    control_pkg = get_package_share_directory('ros_ship_control')
    controller_config = os.path.join(control_pkg, 'config', 'controller.yaml')

    return LaunchDescription([
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='ros_ship',
            output='screen',
            parameters=[controller_config],  # In ROS 2, can be used here too if it includes robot_description
        ),

        # Controller spawner
        Node(
            package='controller_manager',
            executable='spawner.py',
            name='controller_spawner',
            namespace='ros_ship',
            arguments=[
                'joint_state_controller',
                'right_motor_controller',
                'left_motor_controller',
                '--controller-manager-timeout', '50',
            ],
            output='screen',
        ),
    ])
