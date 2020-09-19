cmake_minimum_required(VERSION 3.0.2)
project(ece6460_gps_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_geometry_msgs
  gps_common
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ROS node to demonstrate subscribing to a GPS fix
# message and processing the data
add_executable(gps_transform_example_node
  src/gps_transform_example_node.cpp
  src/GpsTransformExample.cpp
)
add_dependencies(gps_transform_example_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gps_transform_example_node
  ${catkin_LIBRARIES}
)
