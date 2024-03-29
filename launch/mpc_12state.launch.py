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

    # Declare the argument for the node namespace
    mpc_namespace_arg = DeclareLaunchArgument(
        'mpc_namespace',
        default_value='',
        description='Namespace of the mpc_node'
    )

    # MPC input odom topic
    mpc_odom_topic_arg = DeclareLaunchArgument(
        'mpc_odom_topic',
        default_value='interceptor/mavros/local_position/odom',
        description='MPC input odom topic topic'
    )

    # MPC input IMU topic
    mpc_imu_topic_arg = DeclareLaunchArgument(
        'mpc_imu_topic',
        default_value='interceptor/mavros/imu/data',
        description='MPC input IMU topic topic'
    )

    # MPC input reference path topic
    mpc_ref_path_topic_arg = DeclareLaunchArgument(
        'mpc_ref_path_topic',
        default_value='/out/gru_predicted_path',
        description='MPC input reference path topic'
    )

    # MPC output trajectory command topic
    mpc_cmd_topic_arg = DeclareLaunchArgument(
        'mpc_cmd_topic_arg',
        default_value='/interceptor/geometric_controller/multi_dof_setpoint',
        description='MPC output command topic'
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
            ('mpc/in/odom', LaunchConfiguration('mpc_odom_topic')),
            ('mpc/in/imu', LaunchConfiguration('mpc_imu_topic')),
            ('mpc/in/ref_traj', 'mpc/in/ref_traj'),
            ('mpc/in/ref_traj_poses', 'mpc/in/ref_traj_poses'),
            # ('mpc/in/ref_traj_path', 'traj_predictor/const_vel_path'),
            ('mpc/in/ref_traj_path', LaunchConfiguration('mpc_ref_path_topic')),
            ('mpc/out/path', 'mpc/out/path'),
            ('mpc/out/trajectory_command', LaunchConfiguration('mpc_cmd_topic_arg')),
        ],
        # prefix='gdb -ex run --args'
    )

    return LaunchDescription([
        yaml_path_arg,
        mpc_namespace_arg,
        mpc_odom_topic_arg,
        mpc_imu_topic_arg,
        mpc_ref_path_topic_arg,
        mpc_cmd_topic_arg,
        mpc_node,
        LogInfo(msg=["Launching mpc_12state_node with parameters from: ", LaunchConfiguration('yaml_path')]),
    ])
