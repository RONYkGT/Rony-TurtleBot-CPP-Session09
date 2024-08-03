cmake_minimum_required(VERSION 3.8)
project(session_09)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtle_controller_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(driver src/driver_node.cpp)
add_executable(lap_timer_server src/lap_time_action_srv.cpp)
add_executable(lap_timer_client src/lap_time_action_cli.cpp)
add_executable(wall_finder_service src/wall_finder_service.cpp)

ament_target_dependencies(driver rclcpp std_msgs geometry_msgs sensor_msgs turtle_controller_interfaces)
ament_target_dependencies(lap_timer_server
  rclcpp
  rclcpp_action
  nav_msgs
  turtle_controller_interfaces
)
ament_target_dependencies(lap_timer_client
  rclcpp
  rclcpp_action
  turtle_controller_interfaces
)
ament_target_dependencies(wall_finder_service
  rclcpp
  geometry_msgs
  sensor_msgs
  turtle_controller_interfaces
)

install(TARGETS
  driver
  lap_timer_server
  lap_timer_client
  wall_finder_service
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
