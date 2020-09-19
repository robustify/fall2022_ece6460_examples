cmake_minimum_required(VERSION 3.0.2)
project(ece6460_transform_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_example1
  src/transform_example1.cpp
)
add_dependencies(transform_example1
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(transform_example1
  ${catkin_LIBRARIES}
)

add_executable(transform_example2
  src/transform_example2.cpp
)
add_dependencies(transform_example2
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(transform_example2
  ${catkin_LIBRARIES}
)
