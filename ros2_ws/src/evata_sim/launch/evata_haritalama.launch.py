import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='duvarlı_pist')

    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # Dünya dosyasını belirle
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'navigasyon', 'models', 'world', 'arabasız_pist.world')

    # IGN_GAZEBO_RESOURCE_PATH ayarını yap
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=(
            os.path.join("/opt/ros/humble", "share") +
            ":" +
            os.path.join(src_dir, "src", "evata_sim", "navigasyon", "models") +
            ":" +
            os.path.join(src_dir, "src", "evata_sim", "pist_world", "models")
        )
    )

    # Evata aracını spawnla
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', "Evata",
            '-name', "Evata",
            '-file', os.path.join(src_dir, "src", "evata_sim", "pist_world", "models", "Evata", "model.sdf"),
            '-allow_renaming', 'true',
            '-x', '-2.0',
            '-y', '-0.5',
            '-z', '0.01'
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

    # ROS-Ignition Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Velocity command (ROS2 -> IGN)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Odometry (IGN -> ROS2)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # TF (IGN -> ROS2)
            '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Lidar (IGN -> ROS2)
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # IMU (IGN -> ROS2)
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Camera (IGN -> ROS2)
            '/camera/rgb/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    # Static transform publisher for map to odom
    map_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    # Robot state publisher
    sdf = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'models', 'Evata', 'model.sdf')
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': doc.toxml()}]
    )

    # Haritalama için SLAM Toolbox'ı başlat
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/scan', '/scan')]
    )

    # Rviz2'yi başlat
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('evata_sim'), 'config', 'haritalama.rviz')]
    )

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        bridge,
        map_static_tf,
        robot_state_publisher,
        slam_toolbox,
        rviz2,

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
        
    ])
