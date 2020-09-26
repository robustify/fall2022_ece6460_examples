cmake_minimum_required(VERSION 3.0.2)
project(ece6460_launch_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(param_loader
  src/param_loader_node.cpp
  src/ParamLoader.cpp
)
add_dependencies(param_loader
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(param_loader
  ${catkin_LIBRARIES}
)

