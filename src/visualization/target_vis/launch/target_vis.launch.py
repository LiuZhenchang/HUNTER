from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_arg(name: str, default: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default)


def generate_launch_description():
    scale = declare_arg('scale', '150.0')
    target0_x = declare_arg('target0_x', '50.0')
    target0_y = declare_arg('target0_y', '50.0')
    target1_x = declare_arg('target1_x', '10.0')
    target1_y = declare_arg('target1_y', '0.0')
    target2_x = declare_arg('target2_x', '100.0')
    target2_y = declare_arg('target2_y', '0.0')

    target_move_node = Node(
        package='target_vis',
        executable='target_move',
        name='target_move',
        output='screen',
        parameters=[{
            'target0_x': LaunchConfiguration('target0_x'),
            'target0_y': LaunchConfiguration('target0_y'),
            'target1_x': LaunchConfiguration('target1_x'),
            'target1_y': LaunchConfiguration('target1_y'),
            'target2_x': LaunchConfiguration('target2_x'),
            'target2_y': LaunchConfiguration('target2_y'),
        }],
    )

    target_visualization_node = Node(
        package='target_vis',
        executable='target_visualization',
        name='target_visualization',
        output='screen',
        parameters=[{
            'scale': LaunchConfiguration('scale'),
        }],
    )

    return LaunchDescription([
        scale,
        target0_x,
        target0_y,
        target1_x,
        target1_y,
        target2_x,
        target2_y,
        target_move_node,
        target_visualization_node,
    ])


