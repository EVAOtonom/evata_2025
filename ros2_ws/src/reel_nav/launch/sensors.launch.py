from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    def gnome_tab(title, command):
        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab', '-t', title,
                '--', 'bash', '-c', f'{command}; exec bash'
            ],
            output='screen'
        )

    # === İlk aşamada açılacaklar ===
    zed2i = gnome_tab(
        'ZED2i',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zed2i camera_model:=zed2i'
    )
    zedm = gnome_tab(
        'ZEDm',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedm camera_model:=zedm'
    )

    # === 5 saniye gecikmeli açılacaklar ===
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            gnome_tab('Sign Detector', 'ros2 run reel_evata sign_detector'),
            gnome_tab('Lane Detection', 'ros2 run reel_evata laneDetection')
        ]
    )

    return LaunchDescription([
        zed2i,
        zedm,
        delayed_nodes
    ])
