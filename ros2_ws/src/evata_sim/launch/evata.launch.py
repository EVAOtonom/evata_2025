from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='evata_sim',
        #    executable='laneDetection',
        #    name='lane_detection_node',
        #    output='screen'
        #),
        Node(
            package='evata_sim',
            executable='sign_converted',
            name='sign_converted_node',
            output='screen'
        ),
        Node(
            package='evata_sim',
            executable='live_gps',
            name='live_gps_node',
            output='screen'
        ),
    ])

