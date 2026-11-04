"""
全系统启动 (导航模式)
导航主链路:
1. robot_state_publisher + ros2_control
2. Livox + FAST-LIVO2(经兼容桥接) 提供 /fastlio2/lio_odom
3. safe_odom_mux 在 FAST-LIVO2 桥接里程计 / 轮式里程计之间切换并发布 odom -> base_link
4. ICP 提供 map -> odom
5. Nav2 直接接收 RViz 2D Goal Pose
6. 导航避障只使用 Livox / FAST-LIO 点云，不接入 D435
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

import sys
# Nested benchmark launches do not automatically add this launch directory to
# Python's module search path.  Resolve it explicitly so the local helper works
# both when launched directly and when included by another launch file.
nav_launch_dir_for_import = os.path.join(
    get_package_share_directory('swerve_navigation'), 'launch'
)
if nav_launch_dir_for_import not in sys.path:
    sys.path.insert(0, nav_launch_dir_for_import)
from lidar_config_helper import merge_lidar_config

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _expand_path(raw: str) -> str:
    return os.path.abspath(os.path.expandvars(os.path.expanduser(raw)))


def _validate_map_yaml(context, *args, **kwargs):
    resolved = _expand_path(LaunchConfiguration('map_yaml').perform(context))
    if not resolved:
        raise RuntimeError('必须提供 map_yaml')
    if not os.path.isfile(resolved):
        raise RuntimeError(f'map_yaml 不存在或不可读: {resolved}')
    return [SetLaunchConfiguration('map_yaml', resolved)]


def _setup_localization_nodes(context, *args, **kwargs):
    use_gps = LaunchConfiguration('use_gps').perform(context).lower() in ('true', '1', 'yes')
    allow_reverse = LaunchConfiguration('allow_reverse')

    fast_livo_dir = get_package_share_directory('fast_livo')
    nav_dir = get_package_share_directory('swerve_navigation')
    icp_dir = get_package_share_directory('icp_registration')

    nodes = []

    lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_livo_dir, 'launch', 'swerve_lio.launch.py')),
        launch_arguments={
            'namespace': 'fastlio2',
            'core_params_file': os.path.join(fast_livo_dir, 'config', 'livo_navigation.yaml'),
            'bridge_params_file': os.path.join(fast_livo_dir, 'config', 'bridge_navigation.yaml'),
        }.items(),
    )
    nodes.append(lio_node)

    safe_odom_mux = Node(
        package='swerve_navigation',
        executable='safe_odom_mux.py',
        name='safe_odom_mux',
        parameters=[{
            'use_sim_time': False,
            'lio_odom_topic': '/fastlio2/lio_odom',
            'wheel_odom_topic': '/odom',
            'output_odom_topic': '/odom_safe',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_safe_odom_mux')),
    )
    nodes.append(safe_odom_mux)

    cmd_vel_safety_gate = Node(
        package='swerve_navigation',
        executable='cmd_vel_safety_gate.py',
        name='cmd_vel_safety_gate',
        parameters=[{
            'use_sim_time': False,
            'input_cmd_topic': '/cmd_vel_safe_in',
            'output_cmd_topic': '/cmd_vel_safe',
            'hold_topic': '/localization_hold',
            'hold_duration': 1.5,
            'cmd_timeout': 0.5,
            'publish_rate': 30.0,
            # Reverse is disabled by default: rear obstacle coverage has not
            # been independently validated for the work-platform chassis.
            'allow_reverse': ParameterValue(allow_reverse, value_type=bool),
            'reverse_authorization_topic': LaunchConfiguration(
                'reverse_authorization_topic'),
            'reverse_authorization_timeout': ParameterValue(
                LaunchConfiguration('reverse_authorization_timeout'),
                value_type=float),
            'authorized_reverse_max_speed': ParameterValue(
                LaunchConfiguration('authorized_reverse_max_speed'),
                value_type=float),
            'authorized_reverse_lateral_tolerance': ParameterValue(
                LaunchConfiguration('authorized_reverse_lateral_tolerance'),
                value_type=float),
            'authorized_reverse_angular_tolerance': ParameterValue(
                LaunchConfiguration('authorized_reverse_angular_tolerance'),
                value_type=float),
        }],
        output='screen',
    )
    nodes.append(cmd_vel_safety_gate)

    localization_costmap_guard = Node(
        package='swerve_navigation',
        executable='localization_costmap_guard.py',
        name='localization_costmap_guard',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )
    nodes.append(localization_costmap_guard)

    resolved_pcd_path = _expand_path(LaunchConfiguration('pcd_path').perform(context))
    if not resolved_pcd_path:
        raise RuntimeError('必须提供 pcd_path')
    if not os.path.isfile(resolved_pcd_path):
        raise RuntimeError(f'pcd_path 不存在或不可读: {resolved_pcd_path}')

    icp_config = os.path.join(icp_dir, 'config', 'icp.yaml')

    if use_gps:
        resolved_datum_path = _expand_path(LaunchConfiguration('datum_path').perform(context))
        if not resolved_datum_path:
            raise RuntimeError('use_gps=true 时必须提供 datum_path')
        if not os.path.isfile(resolved_datum_path):
            raise RuntimeError(f'datum_path 不存在: {resolved_datum_path}')

        ublox_config = os.path.join(nav_dir, 'config', 'ublox_gps.yaml')
        nodes.append(
            Node(
                package='ublox_gps',
                executable='ublox_gps_node',
                name='ublox_gps_node',
                output='screen',
                parameters=[ublox_config],
                remappings=[('fix', '/gps/fix')],
            )
        )
        nodes.append(
            Node(
                package='swerve_navigation',
                executable='gps_initial_pose.py',
                name='gps_initial_pose',
                parameters=[{'datum_path': resolved_datum_path}],
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='icp_registration',
                executable='icp_registration_node',
                parameters=[
                    icp_config,
                    {
                        'pcd_path': resolved_pcd_path,
                        'use_sim_time': False,
                        'laser_frame_id': 'base_link',
                        'pointcloud_topic': '/fastlio2/body_cloud',
                        'initial_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    },
                ],
                output='screen',
            )
        )
    else:
        import yaml as _yaml

        pose_str = LaunchConfiguration('initial_pose').perform(context)
        try:
            pose_list = _yaml.safe_load(pose_str)
            if not isinstance(pose_list, list) or len(pose_list) != 6:
                raise ValueError
            pose_list = [float(v) for v in pose_list]
        except Exception as exc:
            raise RuntimeError(
                f'initial_pose 必须为 6 元素列表 [x,y,z,roll,pitch,yaw]，当前值: {pose_str}'
            ) from exc

        nodes.append(
            Node(
                package='icp_registration',
                executable='icp_registration_node',
                parameters=[
                    icp_config,
                    {
                        'pcd_path': resolved_pcd_path,
                        'use_sim_time': False,
                        'laser_frame_id': 'base_link',
                        'pointcloud_topic': '/fastlio2/body_cloud',
                        'initial_pose': pose_list,
                    },
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    bringup_dir = get_package_share_directory('swerve_bringup')
    desc_dir = get_package_share_directory('swerve_description')
    nav_dir = get_package_share_directory('swerve_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ---------- 底盘运动学参数 ----------
    with open(os.path.join(desc_dir, 'config', 'chassis_version_6.0.yaml')) as f:
        _cp = yaml.safe_load(f)['chassis']
    _wheel_radius   = str(_cp['wheel_radius'])
    _half_wheelbase = str(_cp['wheelbase'] / 2.0)
    _half_track     = str(_cp['track_width'] / 2.0)

    use_rviz = LaunchConfiguration('use_rviz')
    start_robot_hardware = LaunchConfiguration('start_robot_hardware')
    map_yaml = LaunchConfiguration('map_yaml')
    steering_can = LaunchConfiguration('steering_can_interface')
    driving_can = LaunchConfiguration('driving_can_interface')
    nav_controller_max_wheel_speed = LaunchConfiguration('nav_controller_max_wheel_speed')
    nav_controller_wheel_accel_limit = LaunchConfiguration('nav_controller_wheel_accel_limit')

    urdf_file = os.path.join(desc_dir, 'urdf', 'swerve.urdf.xacro')
    controllers_yaml = os.path.join(bringup_dir, 'config', 'controllers_navigation.yaml')
    controllers_params = RewrittenYaml(
        source_file=controllers_yaml,
        param_rewrites={
            'max_wheel_speed': nav_controller_max_wheel_speed,
            'wheel_accel_limit': nav_controller_wheel_accel_limit,
            'wheel_radius': _wheel_radius,
            'fl_pos_x':  _half_wheelbase,
            'fl_pos_y':  _half_track,
            'fr_pos_x':  _half_wheelbase,
            'fr_pos_y': f'-{_half_track}',
            'bl_pos_x': f'-{_half_wheelbase}',
            'bl_pos_y':  _half_track,
            'br_pos_x': f'-{_half_wheelbase}',
            'br_pos_y': f'-{_half_track}',
        },
        convert_types=True,
    )
    nav2_params = LaunchConfiguration('nav2_params_file')
    bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    bt_through_poses_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    rviz_config = os.path.join(nav_dir, 'rviz', 'navigation_debug.rviz')

    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' steering_can_interface:=', steering_can,
            ' driving_can_interface:=', driving_can,
        ]),
        value_type=str,
    )

    configured_nav_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'yaml_filename': map_yaml,
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_through_poses_xml,
        },
        convert_types=True,
    )
    map_server_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={'yaml_filename': map_yaml},
        convert_types=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
        output='screen',
        condition=IfCondition(start_robot_hardware),
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_params,
        ],
        output='screen',
        condition=IfCondition(start_robot_hardware),
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
        condition=IfCondition(start_robot_hardware),
    )
    sdc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
            '--service-call-timeout', '60.0',
        ],
        output='screen',
    )
    sdc_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[sdc_spawner]),
        condition=IfCondition(start_robot_hardware),
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': merge_lidar_config(os.path.join(
                get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')),
            'cmdline_bd_code': 'livox0000000001',
        }],
    )

    livox_pointcloud_relay = Node(
        package='swerve_navigation',
        executable='pointcloud_qos_relay.py',
        name='livox_pointcloud_relay',
        parameters=[{
            'input_topic': '/fastlio2/body_cloud',
            'output_topic': '/livox/body_cloud_nav2',
            'restamp_on_receive': True,
            'filter_min_z': -0.15,
            'filter_max_z': 1.80,
            # Self-box filter: 去除完整运输姿态机身的 LiDAR 回波。上游桥接
            # 也做分段过滤，但升降柱/收拢手臂边缘仍可能产生少量漏点。
            # track_width=0.52 → half=0.26, 轮子半径 0.075, 转向90°时
            # 轮子外缘达 y=0.26+0.075=0.335m, 留余量取 ±0.44
            # z 下限: 轮子底部在 base_link 下方 -0.235m, 留余量取 -0.30
            'self_filter_enabled': True,
            'self_filter_min_x': -0.45,
            'self_filter_max_x': 0.45,
            'self_filter_min_y': -0.44,
            'self_filter_max_y': 0.44,
            'self_filter_min_z': -0.30,
            'self_filter_max_z': 1.80,
        }],
        output='screen',
    )

    # Keep the final collision-stop input in a separate relay process. It uses
    # the complete height-filtered cloud without planning-only downsampling.
    livox_safety_cloud_relay = Node(
        package='swerve_navigation',
        executable='pointcloud_qos_relay.py',
        name='livox_safety_cloud_relay',
        parameters=[{
            'input_topic': '/fastlio2/body_cloud',
            'output_topic': '/livox/body_cloud_safety',
            'restamp_on_receive': True,
            'filter_min_z': -0.15,
            'filter_max_z': 1.80,
            'self_filter_enabled': True,
            'self_filter_min_x': -0.45,
            'self_filter_max_x': 0.45,
            'self_filter_min_y': -0.44,
            'self_filter_max_y': 0.44,
            'self_filter_min_z': -0.30,
            'self_filter_max_z': 1.80,
        }],
        output='screen',
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/body_cloud_nav2'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': -0.15,
            # Fine alignment must see shelf lips and tabletop edges above the
            # chassis plane, not only low legs and floor-level returns.
            'max_height': 1.80,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )

    local_window_marker = Node(
        package='swerve_navigation',
        executable='local_window_marker.py',
        name='local_window_marker',
        parameters=[{
            'topic': '/local_window_marker',
            'frame_id': 'base_footprint',
            'size_x': 6.0,
            'size_y': 6.0,
            'thickness': 0.01,
            'z_offset': 0.005,
            'alpha': 0.18,
        }],
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_server_params],
    )
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[LaunchConfiguration('collision_monitor_params_file')],
    )
    collision_monitor_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'false',
            'params_file': configured_nav_params,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    localization_ready_gate = Node(
        package='swerve_navigation',
        executable='wait_for_localization_ready.py',
        name='wait_for_localization_ready',
        parameters=[{
            'body_cloud_topic': '/fastlio2/body_cloud',
            'lio_odom_topic': '/fastlio2/lio_odom',
            'localization_topic': '/localization_ok',
            'freshness_sec': 1.0,
            'stable_duration_sec': 1.5,
            'diagnostic_period_sec': 2.0,
        }],
        output='screen',
    )
    def start_nav2_after_localization(event, _context):
        if event.returncode != 0:
            return []
        return [nav2_navigation_launch]

    nav2_after_localization = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_ready_gate,
            on_exit=start_nav2_after_localization,
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='是否启动 RViz'),
        DeclareLaunchArgument(
            'start_robot_hardware',
            default_value='true',
            description='启动底盘 robot_state_publisher、controller_manager 和控制器；整机联合模式设为 false',
        ),
        DeclareLaunchArgument(
            'start_safe_odom_mux',
            default_value='true',
            description='启动统一里程计和 odom->base_link TF；复用整机运行时时设为 false',
        ),
        DeclareLaunchArgument('use_gps', default_value='false', description='室外导航: GPS 自动提供初始位姿 + ICP 精确定位; 室内设为 false'),
        DeclareLaunchArgument('pcd_path', default_value='', description='ICP 参考地图 PCD 文件路径'),
        DeclareLaunchArgument('datum_path', default_value='', description='GPS 基准点文件'),
        DeclareLaunchArgument('map_yaml', default_value='', description='2D 导航地图 YAML 文件路径'),
        DeclareLaunchArgument('steering_can_interface', default_value='can5', description='转向电机 CAN 接口'),
        DeclareLaunchArgument('driving_can_interface', default_value='can4', description='驱动电机 CAN 接口'),
        DeclareLaunchArgument('nav_controller_max_wheel_speed', default_value='1.2',
                              description='导航模式舵轮控制器轮速上限 (m/s)'),
        DeclareLaunchArgument('nav_controller_wheel_accel_limit', default_value='1.0',
                              description='导航模式舵轮控制器轮速变化率上限 (m/s^2)'),
        DeclareLaunchArgument('initial_pose', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', description='ICP 初始位姿 [x, y, z, roll, pitch, yaw]'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件',
        ),
        DeclareLaunchArgument(
            'collision_monitor_params_file',
            default_value=os.path.join(nav_dir, 'config', 'collision_monitor_params.yaml'),
            description='碰撞监控参数文件',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=os.path.join(nav_dir, 'behavior_trees', 'navigate_to_pose_smoothed.xml'),
            description='NavigateToPose 行为树',
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=os.path.join(nav_dir, 'behavior_trees', 'navigate_through_poses_smoothed.xml'),
            description='NavigateThroughPoses 行为树',
        ),
        DeclareLaunchArgument(
            'allow_reverse',
            default_value='false',
            description='是否允许自主倒车；未验证后方感知时保持 false',
        ),
        DeclareLaunchArgument(
            'reverse_authorization_topic',
            default_value='/competition/post_pick_reverse_authorized',
            description='抓取后短距直线后退的心跳授权话题',
        ),
        DeclareLaunchArgument(
            'reverse_authorization_timeout',
            default_value='0.25',
            description='倒车授权心跳失效时间 (s)',
        ),
        DeclareLaunchArgument(
            'authorized_reverse_max_speed',
            default_value='0.06',
            description='授权后退速度绝对值上限 (m/s)',
        ),
        DeclareLaunchArgument(
            'authorized_reverse_lateral_tolerance',
            default_value='0.001',
            description='授权后退允许的横向速度误差 (m/s)',
        ),
        DeclareLaunchArgument(
            'authorized_reverse_angular_tolerance',
            default_value='0.001',
            description='授权后退允许的角速度误差 (rad/s)',
        ),
        OpaqueFunction(function=_validate_map_yaml),
        robot_state_publisher,
        controller_manager,
        jsb_spawner,
        sdc_after_jsb,
        livox_driver,
        livox_pointcloud_relay,
        livox_safety_cloud_relay,
        pointcloud_to_laserscan,
        local_window_marker,
        OpaqueFunction(function=_setup_localization_nodes),
        map_server_node,
        map_server_lifecycle,
        collision_monitor_node,
        collision_monitor_lifecycle,
        localization_ready_gate,
        nav2_after_localization,
        rviz_node,
    ])
