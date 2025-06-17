import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='map')

    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # pist_world klasörüne ve pist.world dosyasına giden yolu oluştur
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'pist.world')

    # IGN_GAZEBO_RESOURCE_PATH ayarını yap
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=(
            os.path.join("/opt/ros/humble", "share") +
            ":" +
            os.path.join(src_dir, "src", "evata_sim", "navigasyon", "models")+
            ":" +
            os.path.join(src_dir, "src", "evata_sim", "pist_world", "models")
            
           
        )
    )

    # Aracı spawnla
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', "Evata",
            '-name', "Evata",
            '-file', os.path.join(src_dir, "src", "evata_sim", "pist_world", "models", "Evata", "model.sdf"),
            '-allow_renaming', 'true',
            '-x', '32.6016',
            '-y', '43.6005',
            '-z', '0.6',
            '-R', '0.0',  # Roll (raw) - 0
            '-P', '0.0',  # Pitch - 0
            '-Y', '3.1'   # Yaw - 0
        ],
    )

    # Dünya spawn et
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-file', world_file, '-allow_renaming', 'false'],
    )

    # Evata sim paketinin paylaşım dizini
    evata_sim_share_dir = get_package_share_directory('evata_sim')

    launch_file_dir = os.path.join(evata_sim_share_dir, 'launch')

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        
        # Ign Gazebo Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
            ),
            launch_arguments=[('gz_args', ['-r -v 3 ' + world_file])],
        ),

        # Parametreler
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'
        ),

        # Diğer Launch Dosyaları
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation2.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

    ])
   

