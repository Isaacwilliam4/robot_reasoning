from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    perception = Node(
        package='camera_pkg',
        executable='perception',
        name='perception_node',
        output='screen'
    )

    control = Node(
        package='camera_pkg',
        executable='control',
        name='control_node',
        output='screen'
    )

    return LaunchDescription([
        perception,
        control
    ])