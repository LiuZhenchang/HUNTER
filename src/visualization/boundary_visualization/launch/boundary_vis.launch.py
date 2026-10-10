from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boundary_visualization',
            executable='boundary_visualization',
            name='boundary_visualizer',
            output='screen',
            parameters=[
                {
                    "polygon_points": [
                        450.0, -1000.0,
                        600.0, -600.0,
                        300.0, 400.0,
                        -600.0, 500.0,
                        -700.0, 0.0
                    ],

                    "no_fly_zone_vertices": [
                        -5.0, -5.0,
                        -5.0,  5.0,
                         5.0,  5.0,
                         5.0, -5.0
                    ],
                    "enable_no_fly_zone": false,
                }
            ]
        )
    ])