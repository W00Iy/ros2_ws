from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    robot_description_content = """
    <robot name="robot">
        <link name="world"/>
    </robot>
    """

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
