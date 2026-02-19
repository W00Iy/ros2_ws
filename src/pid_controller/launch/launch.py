from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Launch-argumenter (kan overstyres fra kommandolinje)
    kp_arg = DeclareLaunchArgument('p', default_value='230.0', description='P gain')
    ki_arg = DeclareLaunchArgument('i', default_value='0.0', description='I gain')
    kd_arg = DeclareLaunchArgument('d', default_value='0.0', description='D gain')

    # (valgfritt) simulator-parametre hvis du vil parametrisere de også
    T_arg = DeclareLaunchArgument('T', default_value='0.15', description='Plant time constant')
    noise_arg = DeclareLaunchArgument('noise', default_value='0.0', description='Noise level')

    pid_node = Node(
        package='pid_controller',
        executable='pid_controller_node',
        name='pid_controller_node',
        output='screen',
        parameters=[{
            'p': LaunchConfiguration('p'),
            'i': LaunchConfiguration('i'),
            'd': LaunchConfiguration('d'),
        }],
    )

    config = os.path.join(
        get_package_share_directory('pid_controller'),
        'config',
        'parameters.yaml'
    )

    joint_sim = Node(
        package='joint_simulator',
        executable='joint_simulator_node',
        name='joint_simulator_node',
        output='screen',
        parameters=[config],
    )

    ref_input = Node(
        package='py_srvcli',
        executable='reference_input_node',
        name='reference_input_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        kp_arg, ki_arg, kd_arg,
        T_arg, noise_arg,
        pid_node,
        joint_sim,
        ref_input
    ])

