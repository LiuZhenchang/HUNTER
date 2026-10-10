from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    length_arg = DeclareLaunchArgument('length_', default_value='15.0')
    time_interval_arg = DeclareLaunchArgument('time_interval', default_value='2.0')
    region_vertices_arg = DeclareLaunchArgument('region_vertices',
        default_value='[-800.0, 0.0,-500.0, -800.0, 500.0, -800.0')  

    grid_data_visualization_node = Node(
        package='grid_data_visualization',
        executable='grid_data_visualization_node',
        name='grid_data_visualization',
        output='screen',
        parameters=[{
            'region_vertices': LaunchConfiguration('region_vertices'), 
            'length_': LaunchConfiguration('length_'),
            'time_interval': LaunchConfiguration('time_interval'),
        }]
    )

    package_share_dir = get_package_share_directory('grid_data_visualization')
    rviz_config_path1 = os.path.join(package_share_dir, 'rviz', 'uncertainty.rviz')
    
    print(f"[DEBUG] Package share directory: {package_share_dir}")
    print(f"[DEBUG] RViz config path 1: {rviz_config_path1}")
    print(f"[DEBUG] RViz config 1 exists: {os.path.exists(rviz_config_path1)}")
    
    rviz1_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_uncertainty',
        arguments=['-d', rviz_config_path1],
        output='screen'
    )

    rviz_config_path2 = os.path.join(package_share_dir, 'rviz', 'probability.rviz')
    print(f"[DEBUG] RViz config path 2: {rviz_config_path2}")
    print(f"[DEBUG] RViz config 2 exists: {os.path.exists(rviz_config_path2)}")
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_probability',
        arguments=['-d', rviz_config_path2],
        output='screen'
    )

    return LaunchDescription([
        # Startparameters
        region_vertices_arg,
        length_arg,
        time_interval_arg,
        grid_data_visualization_node,
        rviz1_node,  
        rviz2_node,  
    ])

