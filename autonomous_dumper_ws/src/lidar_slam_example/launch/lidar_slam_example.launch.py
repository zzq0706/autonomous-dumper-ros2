import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'lidar_slam_example'
    package_dir = get_package_share_directory(package_name)
    
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(package_dir, 'nav2_params.yaml'))
    #nav2_params_file = PathJoinSubstitution([FindPackageShare(package_name), 'nav2_params.yaml'])
    slam_toolbox_params_file = LaunchConfiguration('slam_params_file', default=os.path.join(package_dir, 'mapper_params_online_async.yaml'))
    

    return LaunchDescription({
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
            ),
            launch_arguments={'ROS_IP': '192.168.2.123'}.items(),
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'nav2_unity.rviz')],
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_file
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_toolbox_params_file
            }.items()
        )

    })
