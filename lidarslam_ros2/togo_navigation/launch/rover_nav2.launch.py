"""Minimal Nav2 bringup for the SLAM-backed TOGO/Husky rover.

This launch assumes SLAM is already publishing:
  map -> odom       from graph_based_slam
  odom -> base_link from RKO-LIO

It starts only the navigation servers we need now. The stock Nav2
navigation_launch.py also starts route, collision, and docking servers in
Jazzy, which require extra configuration we do not want for the first rover
milestone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def nav2_node(package, executable, name, params_file, use_sim_time):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    debug_map = LaunchConfiguration('debug_map')
    use_slam_map = LaunchConfiguration('use_slam_map')
    request_initial_map_save = LaunchConfiguration('request_initial_map_save')

    default_params = PathJoinSubstitution([
        FindPackageShare('togo_navigation'),
        'config',
        'nav2_slam_params.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('togo_navigation'),
        'rviz',
        'rover_nav_debug.rviz',
    ])

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    nav2 = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/a300_0000/platform/cmd_vel'),
            SetRemap(src='cmd_vel', dst='/a300_0000/platform/cmd_vel'),
            SetRemap(src='/cmd_vel_smoothed', dst='/a300_0000/platform/cmd_vel'),
            SetRemap(src='cmd_vel_smoothed', dst='/a300_0000/platform/cmd_vel'),
            Node(
                package='togo_navigation',
                executable='debug_map_publisher.py',
                name='debug_map_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(debug_map),
            ),
            Node(
                package='togo_navigation',
                executable='slam_to_occupancy_grid',
                name='slam_to_occupancy_grid',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_cloud_topic': '/modified_map',
                        'output_map_topic': '/map',
                        'target_frame': 'map',
                        'resolution': 0.20,
                        'width_m': 80.0,
                        'height_m': 80.0,
                        'origin_x': -40.0,
                        'origin_y': -40.0,
                        'min_obstacle_height': 0.25,
                        'max_obstacle_height': 1.20,
                        'min_points_per_cell': 1,
                        'initialize_as_free': True,
                        'obstacle_dilation_cells': 0,
                        'enable_gradient_costs': True,
                        'gradient_radius_m': 0.35,
                        'gradient_min_cost': 8,
                        'gradient_power': 1.3,
                        'enable_terrain_hazards': True,
                        'terrain_min_height': -0.75,
                        'terrain_max_height': 1.20,
                        'terrain_slope_hazard_deg': 22.0,
                        'terrain_step_hazard_m': 0.25,
                        'terrain_min_points_per_cell': 5,
                        'terrain_neighbor_radius_cells': 2,
                        'terrain_hazard_dilation_cells': 0,
                        'max_input_range_m': 120.0,
                        'clear_robot_radius_m': 1.2,
                        'robot_frame': 'base_link',
                        'center_map_on_robot': True,
                        'publish_empty_map_until_first_cloud': True,
                    },
                ],
                condition=IfCondition(use_slam_map),
            ),
            Node(
                package='togo_navigation',
                executable='occupancy_grid_to_points',
                name='map_debug_points',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': '/map',
                        'output_topic': '/map_debug_points',
                        'occupied_threshold': 50,
                        'include_unknown': False,
                        'point_z': 0.10,
                    },
                ],
            ),
            Node(
                package='togo_navigation',
                executable='occupancy_grid_to_points',
                name='global_costmap_debug_points',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': '/global_costmap/costmap',
                        'output_topic': '/global_costmap/debug_points',
                        'occupied_threshold': 50,
                        'include_unknown': False,
                        'point_z': 0.16,
                    },
                ],
            ),
            ExecuteProcess(
                cmd=[
                    'bash', '-lc',
                    'source /opt/ros/jazzy/setup.bash; '
                    'source /ws/install/setup.bash; '
                    'until ros2 service type /map_save >/dev/null 2>&1; do sleep 1; done; '
                    'sleep 2; '
                    'ros2 service call /map_save std_srvs/srv/Empty'
                ],
                output='screen',
                condition=IfCondition(request_initial_map_save),
            ),
            nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time),
            nav2_node('nav2_smoother', 'smoother_server', 'smoother_server', params_file, use_sim_time),
            nav2_node('nav2_planner', 'planner_server', 'planner_server', params_file, use_sim_time),
            nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', params_file, use_sim_time),
            nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params_file, use_sim_time),
            nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params_file, use_sim_time),
            nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                    },
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='nav2_rviz',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                condition=IfCondition(rviz),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'debug_map',
            default_value='false',
            description='Publish a simple /map OccupancyGrid until the SLAM-to-OGM mapper exists.',
        ),
        DeclareLaunchArgument(
            'use_slam_map',
            default_value='true',
            description='Convert /modified_map PointCloud2 into /map OccupancyGrid for Nav2.',
        ),
        DeclareLaunchArgument(
            'request_initial_map_save',
            default_value='true',
            description='Call /map_save once after startup so /modified_map feeds the SLAM occupancy mapper.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 params configured for SLAM-provided map->odom and RKO odom->base_link.',
        ),
        nav2,
    ])
