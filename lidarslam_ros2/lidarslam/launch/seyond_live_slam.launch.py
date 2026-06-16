"""Integrated live Seyond RKO-LIO frontend + graph SLAM backend launch.

All frontend and backend tuning parameters live in one YAML by default:
  lidarslam/param/seyond_live_slam.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_param_file = LaunchConfiguration('slam_param_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_save_period = LaunchConfiguration('map_save_period')

    rko_node = Node(
        package='rko_lio',
        executable='online_node',
        name='rko_lio_online_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_param_file,
            {'use_sim_time': use_sim_time},
        ],
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
            slam_param_file,
            {'use_sim_time': use_sim_time},
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
        DeclareLaunchArgument(
            'slam_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml',
            description='Single YAML containing both RKO-LIO frontend and graph SLAM backend parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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