cmake_minimum_required(VERSION 3.8)
project(move_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
set ( DEP
  rclcpp
  rclcpp_action
  nav_msgs
  robot_msgs
  geometry_msgs
  tf2
  sensor_msgs
  nav2_msgs
  rclcpp_components
)
add_library(action_server SHARED
  src/server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server ${DEP})

rclcpp_components_register_node(action_server PLUGIN "Move" EXECUTABLE nav_action_server)

add_executable(convert_parc src/convert_parc.cpp)
ament_target_dependencies(convert_parc ${DEP})


install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS
	convert_parc
	DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch  routes  params DESTINATION share/${PROJECT_NAME}
        )

install(PROGRAMS
	src/pose_manage.py
	src/distance.py
  	src/teleop_keyboard.py
	DESTINATION lib/${PROJECT_NAME}
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
