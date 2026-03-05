import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('joint_description')

    # URDF / Xacro
    model_path = os.path.join(pkg_share, 'urdf', 'joint_model.urdf')
    # Hvis du bruker xacro-fil, bytt til:
    # model_path = os.path.join(pkg_share, 'urdf', 'joint_model.urdf.xacro')

    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model not found: {model_path}")

    robot_description = xacro.process_file(model_path).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )
    # evt GUI:
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen'
    # )

    # RViz config (som oppgaven ber om)
    rviz_config = os.path.join(pkg_share, 'config', 'view_model.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_args,
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])