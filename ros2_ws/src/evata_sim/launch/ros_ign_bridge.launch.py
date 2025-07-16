from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Köprü oluşturacak node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
		'/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
		'/model/Evata/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
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
		'/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        '/helios@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        '/helios/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'


            
            
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
        bridge,
        map_static_tf,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])
