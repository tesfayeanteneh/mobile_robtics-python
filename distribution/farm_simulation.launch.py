from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    leader_robot_description = Command(['xacro ', os.path.join(get_package_share_directory('farm_robot_simulation'), 'urdf', 'leader.urdf')])
    follower1_robot_description = Command(['xacro ', os.path.join(get_package_share_directory('farm_robot_simulation'), 'urdf', 'follower1.urdf')])
    follower2_robot_description = Command(['xacro ', os.path.join(get_package_share_directory('farm_robot_simulation'), 'urdf', 'follower2.urdf')])

    spawn_leader = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'leader_robot'], output='screen')
    spawn_follower1 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'follower1_robot'], output='screen')
    spawn_follower2 = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'follower2_robot'], output='screen')

    return LaunchDescription([
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': leader_robot_description}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': follower1_robot_description}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[{'robot_description': follower2_robot_description}]),
        spawn_leader,
        spawn_follower1,
        spawn_follower2,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('farm_robot_simulation'), 'launch', 'ekf_leader.launch.py')])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('farm_robot_simulation'), 'launch', 'ekf_follower1.launch.py')])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('farm_robot_simulation'), 'launch', 'ekf_follower2.launch.py')])
        ),
        Node(package='farm_robot_simulation', executable='leader_control_node.py', output='screen'),
        Node(package='farm_robot_simulation', executable='follower1_control_node.py', output='screen'),
        Node(package='farm_robot_simulation', executable='follower2_control_node.py', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('farm_robot_simulation'), 'launch', 'rviz.launch.py')])
        ),
    ])

