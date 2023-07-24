cmake_minimum_required(VERSION 3.5)
project(rgbd_slam_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_odom_pub src/fake_odom_pub.cpp)
ament_target_dependencies(fake_odom_pub rclcpp nav_msgs tf2_ros geometry_msgs)

################################################################################
# Build
################################################################################
set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY
    launch
    rviz
    maps
    config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  launch/depth_to_scan_nav.launch.py
  launch/point_cloud_xyz.launch.py
  launch/single_rgbd_mapping.launch.py
  launch/fake_lidar_rgbd_mapping.launch.py
  launch/rgbd_navigation.launch.py
  launch/lidar_rgbd_mapping.launch.py
  launch/lidar_rgbd_navigation.launch.py
  launch/scan_merger_test.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  fake_odom_pub
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)

ament_package()
