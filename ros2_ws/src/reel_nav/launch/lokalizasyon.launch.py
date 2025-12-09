import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    src_dir = dir_path.split('/install')[0]
    sdf_path = os.path.join(src_dir, "src", "reel_nav", 'final_deneme.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_mode = LaunchConfiguration('slam', default='True')

    rviz_config_dir = os.path.join(src_dir, "src", "reel_nav", "config", "nav2_evata_view.rviz")
    lidarslam_param_dir = os.path.join(src_dir, "src","li_slam_ros2" ,"lidarslam_ros2", "lidarslam", "param", "lidarslam.yaml")
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_params_file = os.path.join(src_dir, "src", "reel_nav", "config", "slam_toolbox_params.yaml")


    # Xacro dosyasını oku ve işle
    doc = xacro.parse(open(sdf_path))
    xacro.process_doc(doc)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # Sabit dönüşler
        Node(
              package='tf2_ros',
              executable='static_transform_publisher',
              name='static_tf_map_to_odom',
              output='log',
              arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
              package='tf2_ros',
              executable='static_transform_publisher',
              name='static_tf_odom_to_base',
              output='log',
              arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),

        # joint_state_publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]
        ),
        

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]
        ),
        
  
        #     package='zed_wrapper',
        #     executable='zed_node',
        #     name='zed_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),


    ])

