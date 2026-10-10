from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Obstacle node - publishes static obstacle data
    obstacle_node = Node(
        package='obstacle_vis',
        executable='obstacle_node',
        name='obstacle_node',
        output='screen'
    )
    
    # Obstacle visualization node - subscribes to obstacle data and publishes markers
    obstacle_visualization_node = Node(
        package='obstacle_vis',
        executable='obstacle_visualization',
        name='obstacle_visualization',
        output='screen'
    )
    
    return LaunchDescription([
        obstacle_node,
        obstacle_visualization_node,
    ])
