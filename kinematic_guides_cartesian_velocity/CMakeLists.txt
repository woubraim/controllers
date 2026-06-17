cmake_minimum_required(VERSION 3.8)
project(kinematic_guides_cartesian_velocity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(extender_msgs REQUIRED)
#find_package(apriltag_ros2 REQUIRED)
find_package(visual_servoing REQUIRED)
find_package(controller_interface REQUIRED) 
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/shared_control.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  std_msgs
  robot_interfaces
  Eigen3
  extender_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  #apriltag_ros2
  visual_servoing
  controller_interface
  pluginlib
)

# Register your controller class as a component
rclcpp_components_register_nodes(${PROJECT_NAME}
  "cartesian_velocity_controller::SharedControlVelocityController"
)

# Install the library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the entire config directory (containing controllers.yaml + plugin XML)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch directory
# (redundant install removed; directories installed above)

# Export using pluginlib so the controller_interface can find the custom controller
pluginlib_export_plugin_description_file(
        controller_interface config/kinematic_guides_cartesian.xml)


# Standalone RViz marker publisher node
add_executable(goal_marker_publisher src/goal_marker_publisher.cpp)
ament_target_dependencies(goal_marker_publisher
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  extender_msgs
)

install(TARGETS goal_marker_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()