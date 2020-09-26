cmake_minimum_required(VERSION 3.0.2)
project(ece6460_tf_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_pub_example
  src/tf_pub_example.cpp
  src/TfPubExample.cpp
)
add_dependencies(tf_pub_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tf_pub_example
  ${catkin_LIBRARIES}
)

add_executable(tf_lookup_example
  src/tf_lookup_example.cpp
  src/TfLookupExample.cpp
)
add_dependencies(tf_lookup_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tf_lookup_example
  ${catkin_LIBRARIES}
)

