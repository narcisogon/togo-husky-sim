"""Integrated live Seyond RKO-LIO frontend + graph SLAM backend launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')
    save_dir = LaunchConfiguration('save_dir')
    graph_param_file = LaunchConfiguration('graph_param_file')
    rviz = LaunchConfiguration('rviz')
    map_save_period = LaunchConfiguration('map_save_period')

    rko_node = Node(
        package='rko_lio',
        executable='online_node',
        name='rko_lio_online_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'mode': 'online',
            'lidar_topic': lidar_topic,
            'imu_topic': imu_topic,
            'lidar_frame': lidar_frame,
            'imu_frame': imu_frame,
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'odom_topic': '/rko_lio/odometry',
            'deskew': False,
            'voxel_size': 0.20,
            'double_downsample': False,
            'max_correspondance_distance': 4.0,
            'max_scan_delta_sec': 10.0,
            'min_range': 0.2,
            'max_range': 80.0,
            'publish_deskewed_scan': True,
            'deskewed_scan_topic': '/rko_lio/frame',
            'publish_local_map': True,
            'map_topic': '/rko_lio/local_map',
            'use_sim_time': use_sim_time,
        }],
    )

    xyzi_adapter = ExecuteProcess(
        cmd=[
            'python3', '/scripts/add_intensity_to_cloud.py',
            '--ros-args',
            '-p', 'input_topic:=/rko_lio/frame',
            '-p', 'output_topic:=/rko_lio/frame_xyzi',
            '-p', 'intensity:=1.0',
        ],
        output='screen',
    )

    graph_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            graph_param_file,
            {
                'use_sim_time': use_sim_time,
                'use_odom_input': True,
                'global_frame_id': 'map',
                'map_save_dir': save_dir,
                'submap_distance_threshold': 0.8,
                'debug_flag': True,
            },
        ],
        remappings=[
            ('odom_input', '/rko_lio/odometry'),
            ('cloud_input', '/rko_lio/frame_xyzi'),
        ],
    )


    frontend_path = ExecuteProcess(
        cmd=[
            'python3', '/scripts/odom_to_path.py',
            '--ros-args',
            '-r', '__node:=frontend_path_publisher',
            '-p', 'odom_topic:=/rko_lio/odometry',
            '-p', 'path_topic:=/rko_lio/path',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=12000',
            '-p', 'publish_every_n:=1',
        ],
        output='screen',
    )

    reference_path = ExecuteProcess(
        cmd=[
            'python3', '/scripts/gazebo_pose_to_aligned_path.py',
            '--ros-args',
            '-r', '__node:=reference_path_publisher',
            '-p', 'pose_topic:=/gazebo/dynamic_pose',
            '-p', 'frontend_odom_topic:=/rko_lio/odometry',
            '-p', 'path_topic:=/reference/path',
            '-p', 'model_names:=a300-0000,a300_0000,a300,robot,base_link,chassis_link',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=12000',
            '-p', 'min_distance_m:=0.02',
            '-p', 'extra_yaw_offset_deg:=0.0',
        ],
        output='screen',
    )
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    map_save_pulse = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash; '
            'source /ws/install/setup.bash; '
            'period=${MAP_SAVE_PERIOD:-10}; '
            'until ros2 service list | grep -qx /map_save; do sleep 1; done; '
            'while true; do ros2 service call /map_save std_srvs/srv/Empty; sleep "$period"; done'
        ],
        output='screen',
        additional_env={'MAP_SAVE_PERIOD': map_save_period},
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/a300_0000/sensors/seyond_robin_w/scan/points'),
        DeclareLaunchArgument('imu_topic', default_value='/a300_0000/sensors/seyond_robin_w/imu'),
        DeclareLaunchArgument('lidar_frame', default_value='seyond_robin_w_lidar_frame'),
        DeclareLaunchArgument('imu_frame', default_value='seyond_robin_w_imu_frame'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('save_dir', default_value='/ws/src/lidarslam_ros2/output/husky_seyond_graph'),
        DeclareLaunchArgument('graph_param_file', default_value='/ws/src/lidarslam_ros2/lidarslam/param/lidarslam_mid360_rko_graph.yaml'),
        DeclareLaunchArgument('map_save_period', default_value='10'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rko_node,
        xyzi_adapter,
        graph_node,
        frontend_path,
        reference_path,
        map_to_odom_tf,
        map_save_pulse,
        rviz_node,
    ])