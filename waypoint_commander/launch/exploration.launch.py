from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='waypoint_commander', executable='exploration',
             name='exploration_node', output='screen')
    ])
