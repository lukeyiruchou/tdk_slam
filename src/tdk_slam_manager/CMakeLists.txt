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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(laser_angle_filter_node src/laser_angle_filter.cpp)
ament_target_dependencies(laser_angle_filter_node rclcpp sensor_msgs)

# 保留當 fallback：實機除錯時若 EKF 出問題可以先切回這個 broadcaster
add_executable(odom_tf_broadcaster_node src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster_node rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(robot_pose_publisher_node src/robot_pose_publisher.cpp)
ament_target_dependencies(robot_pose_publisher_node
  rclcpp geometry_msgs tf2 tf2_ros)

add_executable(localization_manager_node src/localization_manager.cpp)
ament_target_dependencies(localization_manager_node
  rclcpp geometry_msgs std_msgs nav2_msgs cartographer_ros_msgs ament_index_cpp)

install(TARGETS
  laser_angle_filter_node
  odom_tf_broadcaster_node
  robot_pose_publisher_node
  localization_manager_node
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
