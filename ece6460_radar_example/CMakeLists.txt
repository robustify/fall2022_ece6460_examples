cmake_minimum_required(VERSION 3.0.2)
project(ece6460_radar_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  conti_radar_msgs
  avs_lecture_msgs
)

generate_dynamic_reconfigure_options(
  cfg/RadarExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(radar_example_node
  src/radar_example_node.cpp
  src/RadarExampleNode.cpp
)
add_dependencies(radar_example_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(radar_example_node
  ${catkin_LIBRARIES}
)

