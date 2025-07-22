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

    # 1. Hemen başlat: ZEDm
    zedm = gnome_tab(
        'ZEDm',
        'ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zedm camera_model:=zedm'
    )

    # 2. 5s sonra: laneDetection
    lane_detection = TimerAction(
        period=5.0,
        actions=[
            gnome_tab('Lane Detection', 'ros2 run reel_evata laneDetection'),

            # 3. 5s sonra: zed2i
            TimerAction(
                period=5.0,
                actions=[
                    gnome_tab(
                        'ZED2i',
                        'ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zed2i camera_model:=zed2i'
                    ),

                    # 4. 5s sonra: sign_detector
                    TimerAction(
                        period=5.0,
                        actions=[
                            gnome_tab('Sign Detector', 'ros2 run reel_evata sign_detector')
                        ]
                    )
                ]
            )
        ]
    )

    return LaunchDescription([
        zedm,
        lane_detection
    ])
