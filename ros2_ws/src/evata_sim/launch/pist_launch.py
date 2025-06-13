import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # pist_world klasörüne ve pist.world dosyasına giden yolu oluştur
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'pist.world')

    # Models klasörüne giden yolu oluştur
    models_path = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'models')

    # Gazebo'nun environment variable'larını ayarla
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
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
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/depth_camera/zed/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloud',
            '/depth_camera/zed/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/depth_camera/zed/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_camera/zed/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # Aracı spawnla
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'Evata',
            '-name', 'Evata',
            '-file', os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'models', 'Evata', 'model.sdf'),
            '-allow_renaming', 'true',
            '-x', '32.6016',
            '-y', '43.6005',
            '-z', '0.5'
        ]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        ignition_spawn_entity
    ])
