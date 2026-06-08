from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('togo_bringup'))
    imu_bridge_config = str(pkg_share / 'config' / 'imu_bridge.yaml')

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='togo_imu_gz_bridge',
        namespace='a300_0000/sensors',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': imu_bridge_config,
        }],
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='togo_imu_static_tf',
        output='screen',
        arguments=[
            '--x', '0.059', '--y', '0.0', '--z', '0.161275',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_0_link',
        ],
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='togo_lidar3d_static_tf',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.15',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar3d_0_sensor_link',
        ],
    )

    return LaunchDescription([imu_bridge, imu_tf, lidar_tf])
