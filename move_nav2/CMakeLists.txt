cmake_minimum_required(VERSION 3.8)
project(move_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set ( DEP
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
)

add_executable(planner_client src/planner_client.cpp)
ament_target_dependencies(planner_client ${DEP})


add_executable(controller_client src/followpath.cpp)
ament_target_dependencies(controller_client ${DEP})


add_executable(convert_twist src/convert_twist.cpp)
ament_target_dependencies(convert_twist ${DEP})


install(TARGETS
	  planner_client
	  controller_client
	  convert_twist
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz params maps DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
    src/label_point.py DESTINATION lib/${PROJECT_NAME})

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
