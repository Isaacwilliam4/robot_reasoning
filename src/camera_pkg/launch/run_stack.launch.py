from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('camera_pkg')
    # world_path = os.path.join(pkg_share, 'worlds', 'depth_camera_world.sdf')

    # gazebo = ExecuteProcess(
    #     cmd=[
    #         'ign', 'gazebo', '-r', world_path
    #     ],
    #     output='screen'
    # )
    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/robot/link/camera/depth/image@sensor_msgs/msg/Image@ignition.msgs.Image'
        ],
        output='screen'
    )

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
        bridge,
        perception,
        control
    ])