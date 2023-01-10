from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    obstacle_avoidance = Node(
        package = 'my_package',
        executable = 'obstacle_avoidance',
        output = 'screen',
        remappings = [
            ('laser_scan', '/dolly/laser_scan'),
            ('cmd_vel', '/dolly/cmd_vel')
        ]
    )

    return LaunchDescription([
        obstacle_avoidance
    ])