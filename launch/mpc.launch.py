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
        'mpc.yaml'
    )

    # Declare the argument for the YAML file path
    yaml_path_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=default_yaml_path,
        description='Path to the YAML file with parameters for the mpc_node'
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
        executable='mpc_node',
        name='mpc_node',
        namespace=LaunchConfiguration('mpc_namespace'),
        output='screen',
        parameters=[LaunchConfiguration('yaml_path')],
        remappings=[
            ('px4_ros/in/odom', 'interceptor/mavros/local_position/odom'),
            ('in/imu', 'interceptor/mavros/imu/data'),
            # ('traj_predictor/in/ref_traj', 'traj_predictor/in/ref_traj'),
            # ('traj_predictor/in/ref_traj_poses', 'traj_predictor/ref_traj_poses'),
            ('mpc_tracker/out/path', 'mpc_tracker/path'),
            ('mpc_tracker/command/trajectory', 'mpc_tracker/command/trajectory'),
            ('traj_predictor/in/ref_traj_path', 'traj_predictor/const_vel_path')
        ],
        # prefix='gdb -ex run --args'
    )

    return LaunchDescription([
        yaml_path_arg,
        mpc_namespace_arg,
        mpc_node,
        LogInfo(msg=["Launching mpc_node with parameters from: ", LaunchConfiguration('yaml_path')]),
    ])
