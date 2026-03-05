import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('joint_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'joint_model.urdf')

    # Les URDF inn i en string (robot_description)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Robot state publisher (publiserer TF basert på robot_description)
    robot_description_content = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="robot"><link name="world"/></robot>'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher (uten GUI). Bytt til 'joint_state_publisher_gui' for sliders:
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Hvis du vil ha GUI (sliders) istedenfor non-gui, bruk dette i stedet:
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )

    # Valgfri: statisk transform fra map -> base_link (beholder RViz Fixed Frame = "map")
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base_link',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    # RViz2 (valgfritt). Hvis du har en rviz-konfig kan du peke på den.
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'view_model.rviz')
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf_node,   # kommenter ut hvis du heller setter fixed frame til base_link i RViz
        rviz_node,
    ])