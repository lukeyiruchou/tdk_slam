cmake_minimum_required(VERSION 3.8)
project(tdk_slam_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(laser_angle_filter_node src/laser_angle_filter.cpp)
ament_target_dependencies(laser_angle_filter_node rclcpp sensor_msgs)
add_executable(odom_tf_broadcaster_node src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster_node rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS
  laser_angle_filter_node
  odom_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

# 這裡加入了 rviz 資料夾，編譯時才會被複製到 share 空間
install(DIRECTORY
  launch
  config
  sim
  urdf
  cartographer_config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()