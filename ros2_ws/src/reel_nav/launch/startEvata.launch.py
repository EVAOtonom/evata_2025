from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1. usb_settings node'u ros2 run ile çalıştırılıyor
    sudo_process = ExecuteProcess(
        cmd=['ros2', 'run', 'reel_nav', 'usb_settings'],
        output='screen'
    )

    # 2. sudo_process tamamlanınca geri kalan süreçler başlatılıyor
    sudo_success_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=sudo_process,
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'reel_evata', 'Aks'],
                    output='screen'
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'launch', 'rslidar_sdk', 'start.py'],
                            output='screen'
                        )
                    ]
                ),
                TimerAction(
                    period=4.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'run', 'reel_evata', 'cmd_vel'],
                            output='screen'
                        )
                    ]
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'launch', 'reel_nav', 'navReel.launch.py'],
                            output='screen',
                            shell=True,
                            emulate_tty=True
                        )
                    ]
                ),
            ]
        )
    )

    return LaunchDescription([
        sudo_process,
        sudo_success_handler
    ])
