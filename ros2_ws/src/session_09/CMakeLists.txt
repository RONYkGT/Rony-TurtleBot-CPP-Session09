cmake_minimum_required(VERSION 3.8)
project(session_09)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(talker src/publisher_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs sensor_msgs)


install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
