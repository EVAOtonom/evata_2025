import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))


    launch_file_dir = "/home/oguzcan/otonom_ws/src/evata_sim/launch"
 
 
    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # pist_world klasörüne ve pist.world dosyasına giden yolu oluştur
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'arabasız_pist.world')

    # Models klasörüne giden yolu oluştur
    models_path = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'models')
    sdf = "/home/oguzcan/otonom_ws/src/evata_sim/pist_world/models/Evata/model.sdf"
    # Gazebo'nun environment variable'larını ayarla
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )
    # ----------------------------------------------------------------------------------------- NAV
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
    # Köprü oluşturacak node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
		'/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
		'/model/Evata/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
		'/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
		'/camera/rgb@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
		'/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/odom/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        '/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
		'/depth_camera/zed/image@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloud',
		'/depth_camera/zed/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
		'/depth_camera/zed/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/depth_camera/zed/image@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'


            
            
        ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),



        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation2.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
    ])
