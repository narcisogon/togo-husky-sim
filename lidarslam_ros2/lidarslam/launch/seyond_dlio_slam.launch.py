"""Integrated live Seyond DLIO frontend + graph SLAM backend launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    slam_param_file = LaunchConfiguration('slam_param_file')
    dlio_param_file = LaunchConfiguration('dlio_param_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    map_save_period = LaunchConfiguration('map_save_period')
    enable_map_save_pulse = LaunchConfiguration('enable_map_save_pulse')
    frontend_path_min_distance = LaunchConfiguration('frontend_path_min_distance')
    publish_static_map_to_odom = LaunchConfiguration('publish_static_map_to_odom')
    timed_cloud_scan_period = LaunchConfiguration('timed_cloud_scan_period')
    timed_cloud_reverse_columns = LaunchConfiguration('timed_cloud_reverse_columns')
    dlio_deskew = LaunchConfiguration('dlio_deskew')

    dlio_base_params = '/ws/src/lidarslam_ros2/direct_lidar_inertial_odometry/cfg/dlio.yaml'
    dlio_runtime_params = '/ws/src/lidarslam_ros2/direct_lidar_inertial_odometry/cfg/params.yaml'

    timed_cloud_adapter = Node(
        package='togo_navigation',
        executable='seyond_cloud_time_adapter',
        name='seyond_cloud_time_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_cloud_topic': '/a300_0000/sensors/seyond_robin_w/scan/points'},
            {'output_cloud_topic': '/a300_0000/sensors/seyond_robin_w/scan/points_timed'},
            {'scan_period_sec': timed_cloud_scan_period},
            {'reverse_column_time': timed_cloud_reverse_columns},
            {'stamp_at_scan_start': True},
            {'drop_non_increasing_stamps': True},
        ],
    )

    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        name='dlio_odom_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            dlio_base_params,
            dlio_runtime_params,
            dlio_param_file,
            {'use_sim_time': use_sim_time},
            {'pointcloud/deskew': ParameterValue(dlio_deskew, value_type=bool)},
        ],
        remappings=[
            ('pointcloud', '/a300_0000/sensors/seyond_robin_w/scan/points_timed'),
            ('imu', '/a300_0000/sensors/seyond_robin_w/imu'),
            ('odom', '/dlio/odometry'),
            ('pose', '/dlio/pose'),
            ('path', '/dlio/path_raw'),
            ('kf_pose', '/dlio/keyframes'),
            ('kf_cloud', '/dlio/keyframe_cloud'),
            ('deskewed', '/dlio/deskewed'),
            ('frontend_diagnostics', '/dlio/frontend_diagnostics'),
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        name='dlio_map_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            dlio_base_params,
            dlio_runtime_params,
            dlio_param_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('keyframes', '/dlio/keyframe_cloud'),
            ('map', '/dlio/map'),
        ],
    )

    dlio_odom_tf = Node(
        package='togo_navigation',
        executable='odometry_to_tf',
        name='dlio_odometry_to_tf',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_topic': '/dlio/odometry'},
            {'parent_frame': 'odom'},
            {'child_frame': 'base_link'},
        ],
    )

    graph_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_param_file,
            {'use_sim_time': use_sim_time},
            {'odom_frame_id': 'odom'},
            {'odom_input_cloud_in_odom_frame': True},
        ],
        remappings=[
            ('odom_input', '/dlio/odometry'),
            ('cloud_input', '/dlio/deskewed'),
        ],
    )

    frontend_path = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/odom_to_path.py',
            '--ros-args',
            '-r', '__node:=dlio_path_publisher',
            '-p', 'odom_topic:=/dlio/odometry',
            '-p', 'path_topic:=/dlio/path_simple',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=12000',
            '-p', 'publish_every_n:=1',
            '-p', ['min_distance_m:=', frontend_path_min_distance],
        ],
        output='screen',
    )

    reference_path = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/gazebo_pose_to_aligned_path.py',
            '--ros-args',
            '-r', '__node:=reference_path_publisher',
            '--params-file', dlio_param_file,
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
        condition=IfCondition(publish_static_map_to_odom),
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
        condition=IfCondition(enable_map_save_pulse),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml',
            description='Backend graph SLAM parameters.',
        ),
        DeclareLaunchArgument(
            'dlio_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml',
            description='Seyond DLIO frontend override parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument('enable_map_save_pulse', default_value='false'),
        DeclareLaunchArgument('frontend_path_min_distance', default_value='0.005'),
        DeclareLaunchArgument(
            'timed_cloud_scan_period',
            default_value='0.0666666667',
            description='Synthetic per-point timing span for the 15 Hz organized Gazebo scan.',
        ),
        DeclareLaunchArgument(
            'timed_cloud_reverse_columns',
            default_value='false',
            description='Reverse synthetic per-column time if the simulated scan ordering is opposite.',
        ),
        DeclareLaunchArgument(
            'dlio_deskew',
            default_value='true',
            description='Enable DLIO per-point deskew. Disable to isolate timing/extrinsic issues during turn tests.',
        ),
        DeclareLaunchArgument(
            'publish_static_map_to_odom',
            default_value='false',
            description='Fallback identity map->odom TF. Keep false when graph_based_slam publishes dynamic correction.',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/ws/src/lidarslam_ros2/lidarslam/rviz/mapping.rviz',
            description='RViz config with /modified_map, /map, paths, and TF displays.',
        ),
        timed_cloud_adapter,
        dlio_odom_node,
        dlio_map_node,
        dlio_odom_tf,
        graph_node,
        frontend_path,
        reference_path,
        map_to_odom_tf,
        map_save_pulse,
        rviz_node,
    ])
