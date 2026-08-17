import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'sentry.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Whether to start RViz2')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'sentry.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, robot_state_publisher, rviz2])
