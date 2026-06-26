from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('togo_bringup'))
    bridge_config = str(pkg_share / 'config' / 'seyond_bridge.yaml')

    seyond_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='seyond_robin_w_gz_bridge',
        namespace='a300_0000/sensors',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': bridge_config,
        }],
    )

    # Clearpath publishes the full robot TF under /a300_0000/tf_static. These
    # global static transforms make external tools such as RKO-LIO robust even
    # when they are launched outside the Clearpath namespace or inside Docker.
    seyond_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='seyond_robin_w_mount_static_tf',
        output='screen',
        arguments=[
            '--x', '0.30', '--y', '0.0', '--z', '0.42',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'seyond_robin_w_link',
        ],
    )

    seyond_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='seyond_robin_w_lidar_static_tf',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'seyond_robin_w_link',
            '--child-frame-id', 'seyond_robin_w_lidar_frame',
        ],
    )

    seyond_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='seyond_robin_w_imu_static_tf',
        output='screen',
        arguments=[
            # Keep this in sync with togo_description.urdf.xacro.
            '--x', '0.15', '--y', '-0.05', '--z', '0.08',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'seyond_robin_w_link',
            '--child-frame-id', 'seyond_robin_w_imu_frame',
        ],
    )

    return LaunchDescription([
        seyond_bridge,
        seyond_mount_tf,
        seyond_lidar_tf,
        seyond_imu_tf,
    ])
