# Author: Ziqi Zhou

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	
    package_name = 'rgbd_slam_example'
    package_dir = get_package_share_directory(package_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    param_dir = LaunchConfiguration('params', default=os.path.join(package_dir, 'config', 'nav2_params_lidar_rgbd.yaml'))
    map_dir = LaunchConfiguration('map', default=os.path.join(package_dir, 'maps', 'map_rgbd.yaml'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'),'rviz','nav2_default_view.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
        	'map': map_dir,
            'use_sim_time': 'true',
            'params_file': param_dir
        }.items()
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': param_dir
        }.items()
        ),
        
        # launch tcp endpoint
    	IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
            ),
            launch_arguments={'ROS_IP': '192.168.2.123'}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            # arguments=['-d', os.path.join(package_dir, 'rviz', 'rgbd.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
            
        Node(
		package='differential_drive_controller',
		executable='differential_drive_controller',
		parameters=[
		{'wheel_distance': 1.865},
		{'track_radius': 0.3}
		]
		),
		
		Node(
		package='camera_tools',
		executable='camera_info_pub',
		parameters=[
		{'name': 'image_topic', 'value': '/depth_image'},
		{'name': 'camera_info_topic', 'value': '/camera_info'},
		{'name': 'field_of_view', 'value': 70}
		]
		),
	
		# transform depth image to base link
		Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'depth_camera', 'depth_camera_trans'],  # 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'parent_frame', 'child_frame'
		),
		
		# transform rgb image to base link
		Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=['0', '0', '0', '-1.57', '0', '-1.57', 'rgb_camera', 'rgb_camera_trans'],  # 'x', 'y', 'z', 'yaw', 'pitch', 'roll', 'parent_frame', 'child_frame'
		),
    ])
