## Description
This a repository of ROS2 navgation stack for a autonomous dumper. Please see another repo for the simulator.


## Environment
### ROS Version
ROS2 Humble

### Dependencies
rtabmap_ros:  
`$ sudo apt install ros-$ROS_DISTRO-rtabmap-ros`
  
image_pipeline:  
`$ sudo apt install ros-${ROS_DISTRO}-image-pipeline`

ros2 navigation related packages:  
`$ sudo apt install ros-$ROS_DISTRO-navigation2` \
`$ sudo apt install ros-$ROS_DISTRO-nav2-bringup` \
`$ sudo apt install ros-$ROS_DISTRO-turtlebot3*`

## Introduction & Quick start
This repo provides three different navigation examples: 2D LIDAR-based, RGBD camera-based and hyrid navigation(LIDAR+camera).

For quickly launching 2D LIDAR-based navigation:  
`$ ros2 launch lidar_slam_example lidar_slam_example.launch.py`

For quickly launching RGBD camera-based navigation:  
`$ ros2 launch rgbd_slam_example single_rgbd_mapping.launch.py`  
or  
`$ ros2 launch rgbd_slam_example fake_lidar_rgbd_mapping.launch.py` 

For quickly launching hyrid navigation:  
`$ ros2 launch rgbd_slam_example lidar_rgbd_mapping.launch.py` 

if you have pre-built maps, you can start navigation directly based the pre-built map, just put the map file in the `rgbd_slam_example/maps` folder,
there are already several maps to use, if you have new, don't forget to change the corresponding map name in launch file.

For launching a RGBD camera-based navigation based on a pre-built map, map file in form of *.db is required, the provided *.db map file is too big and is saved in another shared folder, please ask `adrian.huber@tum.de` for the data base if one need it:  
`$ ros2 launch rgbd_slam_example rgbd_navigation.launch.py`  

For launching a hybrid navigation based on a pre-built map, map file in form of *.pgm and *.yaml is required:  
`$ ros2 launch rgbd_slam_example lidar_rgbd_navigation.launch.py`  



## Other tools
In this repository, there are also other tool provided, such as laser scan merger, point clouds merger, laser scan to point cloud tool,  
check them in `ira_laser_tools` folder, carefully use them.










