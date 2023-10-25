import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'trajectory_generation'
    # Define the path to the default YAML file
    default_yaml_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'mpc_12state.yaml'
    )

    # Declare the argument for the YAML file path
    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=default_yaml_path,
        description='Path to the YAML file with parameters for the mpc_12state_node'
    )

    # Declare the argument for the YAML file path
    mpc_namespace_arg = DeclareLaunchArgument(
        'mpc_namespace',
        default_value='',
        description='Namespace of the mpc_node'
    )

    # Define the node
    mpc_node = Node(
        package=pkg_name,
        executable='mpc_12state_node',
        name='mpc_12state_node',
        namespace=LaunchConfiguration('mpc_namespace'),
        output='screen',
        parameters=[LaunchConfiguration('yaml_path')],
        remappings=[
            ('mpc/in/odom', 'interceptor/mavros/local_position/odom'),
            ('mpc/in/imu', 'interceptor/mavros/imu/data'),
            ('mpc/in/ref_traj', 'traj_predictor/in/ref_traj'),
            ('mpc/in/ref_traj_poses', 'traj_predictor/ref_traj_poses'),
            ('mpc/in/ref_traj_path', 'traj_predictor/const_vel_path'),
            ('mpc/out/path', 'mpc_tracker/path'),
            ('mpc/out/trajectory_commands', 'mpc/out/trajectory_command'),
        ],
        # prefix='gdb -ex run --args'
    )

    return LaunchDescription([
        yaml_path_arg,
        mpc_namespace_arg,
        mpc_node,
        LogInfo(msg=["Launching mpc_12state_node with parameters from: ", LaunchConfiguration('yaml_path')]),
    ])
