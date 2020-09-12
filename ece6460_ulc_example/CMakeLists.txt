cmake_minimum_required(VERSION 3.0.2)
project(ece6460_ulc_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  dataspeed_ulc_msgs
)

generate_dynamic_reconfigure_options(
  cfg/UlcExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ulc_example_node
  src/ulc_example_node.cpp
  src/UlcExampleNode.cpp
)
add_dependencies(ulc_example_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(ulc_example_node
  ${catkin_LIBRARIES}
)

