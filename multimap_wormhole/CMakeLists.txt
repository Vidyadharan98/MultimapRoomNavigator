cmake_minimum_required(VERSION 3.8)
project(multimap_wormhole)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_custom_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(wormhole_creator src/wormhole_creator.cpp)
ament_target_dependencies(wormhole_creator rclcpp my_custom_interfaces nav_msgs geometry_msgs)

add_executable(multi_room_navigator src/multi_room_navigator.cpp)
ament_target_dependencies(multi_room_navigator rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs my_custom_interfaces nav_msgs geometry_msgs)

add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs)

install(DIRECTORY
  launch 
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  wormhole_creator
  multi_room_navigator
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
