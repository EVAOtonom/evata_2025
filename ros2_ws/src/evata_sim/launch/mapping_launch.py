import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='my_map')
    launch_file_dir = "/home/oguzcan/otonom_ws/src/evata_sim/launch"
    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # pist_world klasörüne ve pist.world dosyasına giden yolu oluştur
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'arabasız_pist.world')


    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        "/home/oguzcan/otonom_ws/src/evata_sim/pist_world/models"])
    
   #aracı spawnla
   
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', "Evata",
                   '-name', "Evata",
                   '-file', "/home/oguzcan/otonom_ws/src/evata_sim/pist_world/models/Evata/model.sdf",
                   '-allow_renaming', 'true',
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )


    # Spawn world
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-file', world_file,
                   '-allow_renaming', 'false'],
        )        
        
        
        
    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world_file
                             ])]),
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
 
            
    ])
