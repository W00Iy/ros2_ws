import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare('joint_description'),
        'urdf',
        'joint_model.urdf'
    ])

    robot_description = Command(['xacro', ' ', urdf_path])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])