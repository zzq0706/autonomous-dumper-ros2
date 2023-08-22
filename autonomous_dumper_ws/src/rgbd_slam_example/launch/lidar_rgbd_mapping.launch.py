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
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(package_dir, 'config', 'nav2_params_rgbd.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'base_link',
          'visual_odometry':False,
          'icp_odometry':False,
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          #'subscribe_scan_cloud':True,
          'use_action_for_goal':True,
          'RGBD/CreateOccupancyGrid':'true',
          'qos_image':qos,
          'qos_imu':qos,
          'approx_sync':True,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Grid/RangeMax':'15.0',
          'Grid/CellSize':'0.10',
          'Grid/RayTracing':'false',
          'RGBD/LocalRadius':'15.0',
          'Kp/RoiRatios':'0.0 0.0 0.2 0.2',
          'Optimizer/Strategy':'1',
          #'Mem/RehearsalSimilarity':'0.8',
          'RGBD/OptimizeMaxError':'1.0',
          'subscribe_scan':True,
          'Reg/Strategy':'1',
          'RGBD/OptimizeFromGraphEnd':'false',
          'Grid/Sensor':'2', # true for rgbd projection
          
    }
    
    
    remappings=[
          ('rgb/image', '/rgb_image'),
          ('rgb/camera_info', '/camera_info'),
          ('depth/image', '/depth_image'),
          ('scan_cloud', '/points'),
          #('scan', '/scan'),
          ]
    

    return LaunchDescription([
    	
    	# launch tcp endpoint
    	IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
            ),
            launch_arguments={'ROS_IP': '192.168.2.122'}.items(),
        ),
        
        Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'rgbd.rviz')],
        parameters=[{'use_sim_time':True}]
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
		
		Node(
		package='rgbd_slam_example',
		executable='fake_odom_pub',
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

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch
        
        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings
            ),

        #Node(
            #package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            #parameters=[parameters],
            #remappings=remappings
            #),
            
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
        ),
    ])
