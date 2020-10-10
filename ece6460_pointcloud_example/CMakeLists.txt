cmake_minimum_required(VERSION 3.0.2)
project(ece6460_pointcloud_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  dynamic_reconfigure
  pcl_ros
  avs_lecture_msgs
)

add_compile_options(-std=c++17)

generate_dynamic_reconfigure_options(
  cfg/PointCloudExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(passthrough_example
  src/passthrough_example_node.cpp
  src/PassthroughExample.cpp
)
add_dependencies(passthrough_example
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(passthrough_example
  ${catkin_LIBRARIES}
)

add_executable(cluster_example
  src/cluster_example_node.cpp
  src/ClusterExample.cpp
)
add_dependencies(cluster_example
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(cluster_example
  ${catkin_LIBRARIES}
)
