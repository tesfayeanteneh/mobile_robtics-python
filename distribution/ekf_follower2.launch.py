from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(get_package_share_directory('farm_robot_simulation'), 'config', 'ekf_follower2.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_follower2',
            output='screen',
            parameters=[config]
        )
    ])

