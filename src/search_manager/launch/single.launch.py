from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
import launch_ros.descriptions
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    # Declare launch arguments
    init_x_arg = DeclareLaunchArgument('init_x_')
    init_y_arg = DeclareLaunchArgument('init_y_')
    init_z_arg = DeclareLaunchArgument('init_z_')
    agent_task_arg = DeclareLaunchArgument('agent_task_')
    sensor_type_arg = DeclareLaunchArgument('sensor_type_')
    sensor_pd_arg = DeclareLaunchArgument('sensor_pd_')
    sensor_pf_arg = DeclareLaunchArgument('sensor_pf_')
    agent_num_arg = DeclareLaunchArgument('agent_num_')
    
    # Map parameters
    map_resolution_arg = DeclareLaunchArgument('map_resolution', default_value='3.0')
    region_vertices_arg = DeclareLaunchArgument(
        'region_vertices',
        default_value='[-150.0, -150.0, 150.0, -150.0, 150.0, 150.0, -150.0, 150.0]')
    initial_uav_position_arg = DeclareLaunchArgument('initial_uav_position')
    no_fly_zones_arg = DeclareLaunchArgument(
        'no_fly_zones',
        default_value='-30.0,-20.0,30.0,-20.0,30.0,20.0,-30.0,20.0')
    enable_no_fly_zone_arg = DeclareLaunchArgument('enable_no_fly_zone', default_value='true')
    use_event_trigger_arg = DeclareLaunchArgument('use_event_trigger', default_value='true')
    max_steps_arg = DeclareLaunchArgument('max_steps', default_value='9')
    population_size_arg = DeclareLaunchArgument('population_size', default_value='60')
    generation_max_arg = DeclareLaunchArgument('generation_max', default_value='20')
    crossover_fraction_arg = DeclareLaunchArgument('crossover_fraction', default_value='0.8')
    mutation_rate_arg = DeclareLaunchArgument('mutation_rate', default_value='0.2')
    best_stall_max_arg = DeclareLaunchArgument('best_stall_max', default_value='5')
    elite_count_arg = DeclareLaunchArgument('elite_count', default_value='2')
    cover_length_arg = DeclareLaunchArgument('cover_length', default_value='60.0')
    cover_width_arg = DeclareLaunchArgument('cover_width', default_value='60.0')

    
    
    # Prior target positions
    target_num_arg = DeclareLaunchArgument('target_num', default_value='2')
    target0_x_arg = DeclareLaunchArgument('target0_x', default_value='-100.0')
    target0_y_arg = DeclareLaunchArgument('target0_y', default_value='-100.0')
    target1_x_arg = DeclareLaunchArgument('target1_x', default_value='100.0')
    target1_y_arg = DeclareLaunchArgument('target1_y', default_value='100.0')
    target2_x_arg = DeclareLaunchArgument('target2_x', default_value='0.0')
    target2_y_arg = DeclareLaunchArgument('target2_y', default_value='0.0')
    
    drone_id_arg = DeclareLaunchArgument('drone_id')
    uav_id_arg = DeclareLaunchArgument('uav_id')
    
    # Get package share directories
    so3_control_share_dir = get_package_share_directory('so3_control')
    
    # Quadrotor simulator node
    quadrotor_simulator_node = Node(
        package='so3_quadrotor_simulator',
        executable='so3_quadrotor_simulator',
        name=['drone_', LaunchConfiguration('drone_id'), '_quadrotor_simulator_so3'],
        output='screen',
        parameters=[{
            'rate/odom': 20.0,
            'simulator/init_state_x': LaunchConfiguration('init_x_'),
            'simulator/init_state_y': LaunchConfiguration('init_y_'),
            'simulator/init_state_z': LaunchConfiguration('init_z_'),
        }],
        remappings=[
            ('odom', ['drone_', LaunchConfiguration('drone_id'), '_visual_slam/odom']),
            ('cmd', ['drone_', LaunchConfiguration('drone_id'), '_so3_cmd']),
            ('force_disturbance', ['drone_', LaunchConfiguration('drone_id'), '_force_disturbance']),
            ('moment_disturbance', ['drone_', LaunchConfiguration('drone_id'), '_moment_disturbance']),
        ]
    )
    
    # SO3 control component (using rclcpp_components)
    so3_control_component = launch_ros.descriptions.ComposableNode(
        package='so3_control',
        plugin='SO3ControlComponent',
        name=['drone_', LaunchConfiguration('drone_id'), '_so3_control'],
        parameters=[
            os.path.join(so3_control_share_dir, 'config', 'gains_hummingbird.yaml'),
            os.path.join(so3_control_share_dir, 'config', 'corrections_hummingbird.yaml'),
            {
                'so3_control/init_state_x': LaunchConfiguration('init_x_'),
                'so3_control/init_state_y': LaunchConfiguration('init_y_'),
                'so3_control/init_state_z': LaunchConfiguration('init_z_'),
                'mass': 0.98,
                'use_angle_corrections': False,
                'use_external_yaw': False,
                'gains/rot/z': 1.0,
                'gains/ang/z': 0.1,
            }
        ],
        remappings=[
            ('odom', ['drone_', LaunchConfiguration('drone_id'), '_visual_slam/odom']),
            ('position_cmd', ['drone_', LaunchConfiguration('drone_id'), '_planning/pos_cmd']),
            ('motors', ['drone_', LaunchConfiguration('drone_id'), '_motors']),
            ('corrections', ['drone_', LaunchConfiguration('drone_id'), '_corrections']),
            ('so3_cmd', ['drone_', LaunchConfiguration('drone_id'), '_so3_cmd']),
        ]
    )
    
    so3_control_container = ComposableNodeContainer(
        name=['drone_', LaunchConfiguration('drone_id'), '_so3_control_container'],
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            so3_control_component
        ],
        output='screen'
    )
    
    # Odom visualization node
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', LaunchConfiguration('drone_id'), '_odom_visualization'],
        namespace=['drone_', LaunchConfiguration('drone_id'), '_odom_visualization'],
        output='screen',
        parameters=[{
            'length_': LaunchConfiguration('map_resolution'),
            'drone_num': LaunchConfiguration('agent_num_'),
            'color.a': 1.0,
            'color.r': 0.0,
            'color.g': 0.0,
            'color.b': 0.0,
            'covariance_scale': 100.0,
            'robot_scale': 15.0,
            'traj_scale': 2.0,
            'tf45': False,
            'drone_id': LaunchConfiguration('drone_id'),
            'region_vertices': LaunchConfiguration('region_vertices'),
        }],
        remappings=[
            ('odom', ['drone_', LaunchConfiguration('drone_id'), '_visual_slam/odom']),
        ]
    )
    
    # Search manager node
    search_manager_node = Node(
        package='search_manager',
        executable='search_manager',
        name=['drone_', LaunchConfiguration('drone_id'), '_search_manager_node'],
        output='screen',
        parameters=[{
            # Agent parameters
            'agent_id': LaunchConfiguration('drone_id'),
            'agent_task': LaunchConfiguration('agent_task_'),
            'agent_num': LaunchConfiguration('agent_num_'),
            'history_size': 30,
            'init_x_': LaunchConfiguration('init_x_'),
            'init_y_': LaunchConfiguration('init_y_'),
            'init_z_': LaunchConfiguration('init_z_'),
            'use_real_uav': False,
            'uav_id': LaunchConfiguration('uav_id'),
            'use_init_as_takeoff_position': False,
            # DMPC parameters
            'dmpc/safe_uav': 1.0,
            'dmpc/safe_obs': 1.5,
            'dmpc/apf_distance': 10.0,
            'dmpc/apf_param1': 0.6e-2,
            'dmpc/apf_param2': 12.0,
            'dmpc/predict_horizon': 9,
            'dmpc/max_steps': LaunchConfiguration('max_steps'),
            'dmpc/population_size': LaunchConfiguration('population_size'),
            'dmpc/generation_max': LaunchConfiguration('generation_max'),
            'dmpc/crossover_fraction': LaunchConfiguration('crossover_fraction'),
            'dmpc/mutation_rate': LaunchConfiguration('mutation_rate'),
            'dmpc/best_stall_max': LaunchConfiguration('best_stall_max'),
            'dmpc/elite_count': LaunchConfiguration('elite_count'),
            'dmpc/w_jA1': 0.8,
            'dmpc/w_jA2': 0.2,
            'dmpc/w_jB': 1.0,
            'dmpc/w_jC': 3.0,
            'dmpc/w_jD': 2.3,
            'dmpc/k_wJA1': 1.0,
            'dmpc/k_wJA2': 1.0,
            'dmpc/k_wJB': 0.0,
            'dmpc/k_wJC': 1.0,
            'dmpc/k_wJD': 1.0,
            'dmpc/safe_margin_x': 3.2,
            'dmpc/safe_margin_y': 3.2,
            'dmpc/max_dt': 1.0,
            'dmpc/search_jump': 1.0,
            'dmpc/velocity_threshold': 6.0,
            'dmpc/dis_tar_warnning': 0.0,
            'dmpc/target_switch_distance': 80.0,
            'dmpc/target_arrival_distance': 1.0,
            'target_completion/arrival_distance': 1.0,
            'target_completion/consecutive_samples': 3,
            'dmpc/use_expert_system': False,
            'dmpc/use_initial_task_assignment': True,
            'dmpc/enable_no_fly_zone': LaunchConfiguration('enable_no_fly_zone'),
            # Map parameters
            'map/resolution': LaunchConfiguration('map_resolution'),
            'no_fly_zones': LaunchConfiguration('no_fly_zones'),
            'initial_uav_position': LaunchConfiguration('initial_uav_position'),
            'map/map_size_z': 0.0,
            # Sensor property parameters
            'length_': LaunchConfiguration('map_resolution'),
            'seeker_type': 1,
            'cover_length': LaunchConfiguration('cover_length'),
            'cover_width': LaunchConfiguration('cover_width'),
            # Prior target position info
            'use_target_prior': True,
            'target_num': LaunchConfiguration('target_num'),
            'target0_x': PythonExpression(['float(', LaunchConfiguration('target0_x'), ')']),
            'target0_y': PythonExpression(['float(', LaunchConfiguration('target0_y'), ')']),
            'target1_x': PythonExpression(['float(', LaunchConfiguration('target1_x'), ')']),
            'target1_y': PythonExpression(['float(', LaunchConfiguration('target1_y'), ')']),
            'target2_x': PythonExpression(['float(', LaunchConfiguration('target2_x'), ')']),
            'target2_y': PythonExpression(['float(', LaunchConfiguration('target2_y'), ')']),
            'c_n': 0.3,
            'v_n': 20.0,
            # Detect parameters
            'sensor/type': LaunchConfiguration('sensor_type_'),
            'sensor/pd': LaunchConfiguration('sensor_pd_'),
            'sensor/pf': LaunchConfiguration('sensor_pf_'),
            # Plot parameters
            'show_search_result': True,
            'plot_on_all_uavs': False,
            'plot_interval': 0.5,
            'search_time': 500.0,
            # Task parameters
            'task_param/max_trajectory_iterations': 1000,
            'task_param/use_event_trigger': LaunchConfiguration('use_event_trigger'),
            'task_param/rhoMin': 40.0,
            'task_param/path_points_per_execution': 9,
            'task_param/fixed_period_execution_points': 3,
            'task_param/ga_start_time': 0.5,
            'task_param/distance_to_next_target': 1.0,
            'task_param/max_task_time': 10000.0,
            'event_trigger/window_size': 3,
            'event_trigger/execution_stride': 3,
            'event_trigger/probability_weight': 0.4,
            'event_trigger/uncertainty_weight': 0.3,
            'event_trigger/probability_delta_weight': 0.2,
            'event_trigger/uncertainty_delta_weight': 0.1,
            'event_trigger/threshold': 0.25,
            'region_vertices': LaunchConfiguration('region_vertices'),
        }],
        remappings=[
            ('odom', ['drone_', LaunchConfiguration('drone_id'), '_visual_slam/odom']),
            ('position_cmd', ['drone_', LaunchConfiguration('drone_id'), '_planning/pos_cmd']),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        init_x_arg,
        init_y_arg,
        init_z_arg,
        agent_task_arg,
        sensor_type_arg,
        sensor_pd_arg,
        sensor_pf_arg,
        agent_num_arg,
        map_resolution_arg,
        initial_uav_position_arg,
        target_num_arg,
        target0_x_arg,
        target0_y_arg,
        target1_x_arg,
        target1_y_arg,
        target2_x_arg,
        target2_y_arg,
        drone_id_arg,
        uav_id_arg,
        region_vertices_arg,
        no_fly_zones_arg,
        enable_no_fly_zone_arg,
        # Nodes
        quadrotor_simulator_node,
        so3_control_container,
        odom_visualization_node,
        search_manager_node,
        # Communication
        use_event_trigger_arg,
        max_steps_arg,
        population_size_arg,
        generation_max_arg,
        crossover_fraction_arg,
        mutation_rate_arg,
        best_stall_max_arg,
        elite_count_arg,
        cover_length_arg,
        cover_width_arg,
    ])
