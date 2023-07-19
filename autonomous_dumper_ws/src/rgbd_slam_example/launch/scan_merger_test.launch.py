import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():

    package_name = 'rgbd_slam_example'
    package_dir = get_package_share_directory(package_name)

    

    return LaunchDescription([
    	
    	# launch tcp endpoint
    	IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
            ),
            launch_arguments={'ROS_IP': '192.168.2.122'}.items(),
        ),
        
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
		
		# launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth_image'),
                                ('camera_info', '/camera_info'),
                                ('image', '/depth_image')],
                ),
            ],
            output='screen',
        ),
		
		Node(
        package="ira_laser_tools",
        executable="laserscan2pcl",
        name="laserscan_to_pcl",
        parameters=[
        {
			'destination_frame': 'base_scan',
			'cloud_destination_topic': '/scan_cloud',
			'laserscan_topic': '/lidar_scan',
    	}],
        output="both",
        # respawn=True,
        # respawn_delay=2,
    	),
    	
    	Node(
        package="ira_laser_tools",
        executable="pcl_merger",
        name="pcl_merger",
        parameters=[
        {
			'destination_frame': 'base_link',
			'cloud_destination_topic': '/merged_cloud',
			'cloud_topics': '/scan_cloud /points',
    	}],
        output="both",
        # respawn=True,
        # respawn_delay=2,
    	),
    	
    	launch_ros.actions.Node(
            package='rviz2', node_executable='rviz2', output='screen',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'pl_merger_test.rviz')]),

    ])
