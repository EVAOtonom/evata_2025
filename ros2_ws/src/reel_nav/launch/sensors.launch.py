from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # usb_settings + Aks aynı sekmede
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab', '-t', 'USB & Aks',
                '--', 'bash', '-c',
                'ros2 run reel_nav usb_settings && ros2 run reel_evata Aks; exec bash'
            ],
            output='screen'
        ),

        # cmd_vel ayrı sekmede
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab', '-t', 'cmd_vel',
                '--', 'bash', '-c',
                'ros2 run reel_evata cmd_vel; exec bash'
            ],
            output='screen'
        ),

        # nav2 ayrı sekmede
        ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab', '-t', 'Nav2',
                '--', 'bash', '-c',
                'ros2 launch reel_nav navReel.launch.py; exec bash'
            ],
            output='screen'
        ),
    ])
