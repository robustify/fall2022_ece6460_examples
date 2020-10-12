cmake_minimum_required(VERSION 3.0.2)
project(ece6460_vector_to_quat)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  dynamic_reconfigure
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/VectorToQuat.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vector_to_quat
  src/vector_to_quat_node.cpp
  src/VectorToQuat.cpp
)
add_dependencies(vector_to_quat
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(vector_to_quat
  ${catkin_LIBRARIES}
)

