from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vtol',
            executable='cam_node',
            name='cam_node',
            output='screen'
        ),
        Node(
            package='vtol',
            executable='precision_hover',
            name='precision_hover',
            output='screen'
        ),
    ])
