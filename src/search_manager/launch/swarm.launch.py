from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_no_fly_zones_from_yaml(config_path):
    """Load no-fly-zone configuration from a YAML file."""
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        if config and 'no_fly_zones' in config:
            return config['no_fly_zones']
    except Exception as e:
        print(f"Warning: Failed to load no-fly zones config from {config_path}: {e}")
    return None


def generate_launch_description():
    search_manager_share_dir = get_package_share_directory('search_manager')
    default_no_fly_zones_config = os.path.join(search_manager_share_dir, 'config', 'no_fly_zones.yaml')

    # GA parameters
    population_size_arg = DeclareLaunchArgument('population_size', default_value='60')
    generation_max_arg = DeclareLaunchArgument('generation_max', default_value='20')
    crossover_fraction_arg = DeclareLaunchArgument('crossover_fraction', default_value='0.8')
    mutation_rate_arg = DeclareLaunchArgument('mutation_rate', default_value='0.2')
    best_stall_max_arg = DeclareLaunchArgument('best_stall_max', default_value='5')
    elite_count_arg = DeclareLaunchArgument('elite_count', default_value='2')
    max_steps_arg = DeclareLaunchArgument('max_steps', default_value='9')

    initial_uav_position_arg = DeclareLaunchArgument(
        'initial_uav_position',
        default_value='[0.0, -120.0, -120.0, 1.0, 120.0, -120.0, 2.0, 120.0, 120.0, 3.0, -120.0, 120.0]')
    region_vertices_arg = DeclareLaunchArgument(
        'region_vertices',
        default_value='[-150.0, -150.0, 150.0, -150.0, 150.0, 150.0, -150.0, 150.0]')

    no_fly_zones_config_arg = DeclareLaunchArgument('no_fly_zones_config', default_value=default_no_fly_zones_config)
    enable_no_fly_zone_arg = DeclareLaunchArgument('enable_no_fly_zone', default_value='true')
    use_event_trigger_arg = DeclareLaunchArgument('use_event_trigger', default_value='false')

    no_fly_zones_list = [
        {'vertices': [-30.0, -20.0, 30.0, -20.0, 30.0, 20.0, -30.0, 20.0]}
    ]

    cover_length_arg = DeclareLaunchArgument('cover_length', default_value='20.0')
    cover_width_arg = DeclareLaunchArgument('cover_width', default_value='40.0')

    no_fly_zones_str = ""
    for i, zone in enumerate(no_fly_zones_list):
        if i > 0:
            no_fly_zones_str += ";"
        vertices = zone.get('vertices', [])
        no_fly_zones_str += ",".join(str(v) for v in vertices)
    
    # Map parameters
    map_resolution_arg = DeclareLaunchArgument('map_resolution', default_value='3.0')
    drone_count_arg = DeclareLaunchArgument('drone_count', default_value='4')
    
    # Target parameters
    target_num_arg = DeclareLaunchArgument('target_num', default_value='2')
    target0_x_arg = DeclareLaunchArgument('target0_x', default_value='-100.0')
    target0_y_arg = DeclareLaunchArgument('target0_y', default_value='-100.0')
    target1_x_arg = DeclareLaunchArgument('target1_x', default_value='100.0')
    target1_y_arg = DeclareLaunchArgument('target1_y', default_value='100.0')
    target2_x_arg = DeclareLaunchArgument('target2_x', default_value='0.0')
    target2_y_arg = DeclareLaunchArgument('target2_y', default_value='0.0')
    target_scale_arg = DeclareLaunchArgument('target_scale', default_value='10.0')
    
    # Include single.launch.py for drone 0
    single_launch_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('search_manager'),
                'launch',
                'single.launch.py'
            ])
        ]),
        launch_arguments={
            'map_resolution': LaunchConfiguration('map_resolution'),
            'region_vertices': LaunchConfiguration('region_vertices'),
            'no_fly_zones': no_fly_zones_str,
            'enable_no_fly_zone': LaunchConfiguration('enable_no_fly_zone'),
            'use_event_trigger': LaunchConfiguration('use_event_trigger'),
            'initial_uav_position': LaunchConfiguration('initial_uav_position'),
            'target_num': LaunchConfiguration('target_num'),
            'target0_x': LaunchConfiguration('target0_x'),
            'target0_y': LaunchConfiguration('target0_y'),
            'target1_x': LaunchConfiguration('target1_x'),
            'target1_y': LaunchConfiguration('target1_y'),
            'target2_x': LaunchConfiguration('target2_x'),
            'target2_y': LaunchConfiguration('target2_y'),
            'drone_id': '0',
            'uav_id': 'uav1',
            'agent_task_': '1',
            'init_x_': '-120.0',
            'init_y_': '-120.0',
            'init_z_': '10.0',
            'sensor_type_': '1',
            'sensor_pd_': '0.80',
            'sensor_pf_': '0.00',
            'agent_num_': LaunchConfiguration('drone_count'),
            'max_steps': LaunchConfiguration('max_steps'),
            'population_size': LaunchConfiguration('population_size'),
            'generation_max': LaunchConfiguration('generation_max'),
            'crossover_fraction': LaunchConfiguration('crossover_fraction'),
            'mutation_rate': LaunchConfiguration('mutation_rate'),
            'best_stall_max': LaunchConfiguration('best_stall_max'),
            'elite_count': LaunchConfiguration('elite_count'),
            'cover_length': LaunchConfiguration('cover_length'),
            'cover_width': LaunchConfiguration('cover_width'),
        }.items()
    )
    
    
    # Include single.launch.py for drone 1
    single_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('search_manager'),
                'launch',
                'single.launch.py'
            ])
        ]),
        launch_arguments={
            'map_resolution': LaunchConfiguration('map_resolution'),
            'region_vertices': LaunchConfiguration('region_vertices'),
            'no_fly_zones': no_fly_zones_str,
            'enable_no_fly_zone': LaunchConfiguration('enable_no_fly_zone'),
            'use_event_trigger': LaunchConfiguration('use_event_trigger'),
            'region_vertices': LaunchConfiguration('region_vertices'),
            'initial_uav_position': LaunchConfiguration('initial_uav_position'),
            'target_num': LaunchConfiguration('target_num'),
            'target0_x': LaunchConfiguration('target0_x'),
            'target0_y': LaunchConfiguration('target0_y'),
            'target1_x': LaunchConfiguration('target1_x'),
            'target1_y': LaunchConfiguration('target1_y'),
            'target2_x': LaunchConfiguration('target2_x'),
            'target2_y': LaunchConfiguration('target2_y'),
            'drone_id': '1',
            'uav_id': 'uav2',
            'agent_task_': '1',
            'init_x_': '120.0',
            'init_y_': '-120.0',
            'init_z_': '10.0',
            'sensor_type_': '1',
            'sensor_pd_': '0.80',
            'sensor_pf_': '0.00',
            'agent_num_': LaunchConfiguration('drone_count'),
            'max_steps': LaunchConfiguration('max_steps'),
            'population_size': LaunchConfiguration('population_size'),
            'generation_max': LaunchConfiguration('generation_max'),
            'crossover_fraction': LaunchConfiguration('crossover_fraction'),
            'mutation_rate': LaunchConfiguration('mutation_rate'),
            'best_stall_max': LaunchConfiguration('best_stall_max'),
            'elite_count': LaunchConfiguration('elite_count'),
            'cover_length': LaunchConfiguration('cover_length'),
            'cover_width': LaunchConfiguration('cover_width'),
        }.items()
    )

    # Include single.launch.py for drone 2
    single_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('search_manager'),
                'launch',
                'single.launch.py'
            ])
        ]),
        launch_arguments={
            'map_resolution': LaunchConfiguration('map_resolution'),
            'region_vertices': LaunchConfiguration('region_vertices'),
            'no_fly_zones': no_fly_zones_str,
            'enable_no_fly_zone': LaunchConfiguration('enable_no_fly_zone'),
            'use_event_trigger': LaunchConfiguration('use_event_trigger'),
            'initial_uav_position': LaunchConfiguration('initial_uav_position'),
            'target_num': LaunchConfiguration('target_num'),
            'target0_x': LaunchConfiguration('target0_x'),
            'target0_y': LaunchConfiguration('target0_y'),
            'target1_x': LaunchConfiguration('target1_x'),
            'target1_y': LaunchConfiguration('target1_y'),
            'target2_x': LaunchConfiguration('target2_x'),
            'target2_y': LaunchConfiguration('target2_y'),
            'drone_id': '2',
            'uav_id': 'uav3',
            'agent_task_': '1',
            'init_x_': '120.0',
            'init_y_': '120.0',
            'init_z_': '10.0',
            'sensor_type_': '1',
            'sensor_pd_': '0.80',
            'sensor_pf_': '0.00',
            'agent_num_': LaunchConfiguration('drone_count'),
            'max_steps': LaunchConfiguration('max_steps'),
            'population_size': LaunchConfiguration('population_size'),
            'generation_max': LaunchConfiguration('generation_max'),
            'crossover_fraction': LaunchConfiguration('crossover_fraction'),
            'mutation_rate': LaunchConfiguration('mutation_rate'),
            'best_stall_max': LaunchConfiguration('best_stall_max'),
            'elite_count': LaunchConfiguration('elite_count'),
            'cover_length': LaunchConfiguration('cover_length'),
            'cover_width': LaunchConfiguration('cover_width'),
        }.items()
    )

    # Include single.launch.py for drone 3
    single_launch_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('search_manager'),
                'launch',
                'single.launch.py'
            ])
        ]),
        launch_arguments={
            'map_resolution': LaunchConfiguration('map_resolution'),
            'region_vertices': LaunchConfiguration('region_vertices'),
            'no_fly_zones': no_fly_zones_str,
            'enable_no_fly_zone': LaunchConfiguration('enable_no_fly_zone'),
            'use_event_trigger': LaunchConfiguration('use_event_trigger'),
            'initial_uav_position': LaunchConfiguration('initial_uav_position'),
            'target_num': LaunchConfiguration('target_num'),
            'target0_x': LaunchConfiguration('target0_x'),
            'target0_y': LaunchConfiguration('target0_y'),
            'target1_x': LaunchConfiguration('target1_x'),
            'target1_y': LaunchConfiguration('target1_y'),
            'target2_x': LaunchConfiguration('target2_x'),
            'target2_y': LaunchConfiguration('target2_y'),
            'drone_id': '3',
            'uav_id': 'uav4',
            'agent_task_': '1',
            'init_x_': '-120.0',
            'init_y_': '120.0',
            'init_z_': '100.0',
            'sensor_type_': '1',
            'sensor_pd_': '0.80',
            'sensor_pf_': '0.00',
            'agent_num_': LaunchConfiguration('drone_count'),
            'max_steps': LaunchConfiguration('max_steps'),
            'population_size': LaunchConfiguration('population_size'),
            'generation_max': LaunchConfiguration('generation_max'),
            'crossover_fraction': LaunchConfiguration('crossover_fraction'),
            'mutation_rate': LaunchConfiguration('mutation_rate'),
            'best_stall_max': LaunchConfiguration('best_stall_max'),
            'elite_count': LaunchConfiguration('elite_count'),
            'cover_length': LaunchConfiguration('cover_length'),
            'cover_width': LaunchConfiguration('cover_width'),
        }.items()
    )


    # Detect visualization node
    detect_visualization_node = Node(
        package='traj_visualization',
        executable='detect_visualization',
        name='trajectory_visualization',
        output='screen',
        parameters=[{
            'drone_count': LaunchConfiguration('drone_count'),
            'grid_topic': '/trajectory_grid',
            'length_': LaunchConfiguration('map_resolution'),
            'seeker_type': 1,
            'cover_length': LaunchConfiguration('cover_length'),
            'cover_width': LaunchConfiguration('cover_width'),
            'region_vertices': LaunchConfiguration('region_vertices'),
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(search_manager_share_dir, 'config', 'rviz.rviz')],
        name='rviz',
        output='screen'
    )
    
    # Include grid_visualization launch
    grid_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('grid_data_visualization'),
                'launch',
                'grid_visualization.launch.py'
            ])
        ]),
        launch_arguments={
            'region_vertices': LaunchConfiguration('region_vertices'),
            'length_': LaunchConfiguration('map_resolution'),
            'time_interval': '2.0',
        }.items()
    )
    
    # Target visualization nodes
    target_move_node = Node(
        package='target_vis',
        executable='target_move',
        name='target_move',
        output='screen',
        parameters=[{
            'target_num': LaunchConfiguration('target_num'),
            'target0_x': PythonExpression(['float(', LaunchConfiguration('target0_x'), ')']),
            'target0_y': PythonExpression(['float(', LaunchConfiguration('target0_y'), ')']),
            'target1_x': PythonExpression(['float(', LaunchConfiguration('target1_x'), ')']),
            'target1_y': PythonExpression(['float(', LaunchConfiguration('target1_y'), ')']),
            'target2_x': PythonExpression(['float(', LaunchConfiguration('target2_x'), ')']),
            'target2_y': PythonExpression(['float(', LaunchConfiguration('target2_y'), ')']),
        }],
    )
    
    target_visualization_node = Node(
        package='target_vis',
        executable='target_visualization',
        name='target_visualization',
        output='screen',
        parameters=[{
            'scale': PythonExpression(['float(', LaunchConfiguration('target_scale'), ')']),
        }],
    )

    # Obstacle visualization nodes
    obstacle_node = Node(
        package='obstacle_vis',
        executable='obstacle_node',
        name='obstacle_node',
        output='screen'
    )
    
    obstacle_visualization_node = Node(
        package='obstacle_vis',
        executable='obstacle_visualization',
        name='obstacle_visualization',
        output='screen'
    )

    # Boundary visualization nodes
    boundary_visualization_node = Node(
        package='boundary_visualization',
        executable='boundary_visualization',
        name='boundary_visualizer',
        output='screen',
        parameters=[
            {
                "polygon_points": LaunchConfiguration('region_vertices'),
                "no_fly_zones": no_fly_zones_str,
                "enable_no_fly_zone": LaunchConfiguration('enable_no_fly_zone'),
                "line_width": 2.0
            }
        ]
    )

    return LaunchDescription([
        # GA parameters
        population_size_arg,
        generation_max_arg,
        crossover_fraction_arg,
        mutation_rate_arg,
        best_stall_max_arg,
        elite_count_arg,
        max_steps_arg,
        cover_length_arg,
        cover_width_arg,
        # Map parameters
        map_resolution_arg,
        drone_count_arg,
        region_vertices_arg,
        no_fly_zones_config_arg,
        enable_no_fly_zone_arg,
        use_event_trigger_arg,
        initial_uav_position_arg,
        # Target parameters
        target_num_arg,
        target0_x_arg,
        target0_y_arg,
        target1_x_arg,
        target1_y_arg,
        target2_x_arg,
        target2_y_arg,
        target_scale_arg,
        # Include single launch for drone 0
        single_launch_0,
        # single_launch_1,
        # single_launch_2,
        # single_launch_3,
        # Visualization nodes
        detect_visualization_node,  # Enabled detect_visualization using traj_visualization package
        obstacle_node,  # Obstacle data publisher
        obstacle_visualization_node,  # Obstacle markers visualization
        rviz_node,
        boundary_visualization_node,
        # Target visualization nodes
        target_move_node,
        target_visualization_node,
        # GridData visualization (includes grid_data_visualization_node + 2 RViz windows)
        grid_visualization_launch,
    ])
