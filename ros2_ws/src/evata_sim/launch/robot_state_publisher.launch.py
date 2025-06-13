import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

     # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar
    
    # model dosyasının yolu
    sdf_path = os.path.join(src_dir, "src","evata_sim",'pist_world', 'models', 'Evata', 'model.sdf')

    # Xacro dosyasını oku ve işleme yap
    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]),
    ])

