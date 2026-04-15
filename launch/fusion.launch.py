from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='od_fusion',
            executable='od_fusion_node',
            name='od_fusion_node',
            output='screen',
            parameters=[],
        ),
    ])
