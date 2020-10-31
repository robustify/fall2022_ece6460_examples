cmake_minimum_required(VERSION 3.0.2)
project(ece6460_opencv_example)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  cv_bridge
  roscpp
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/HsvExample.cfg
)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hsv_example_node
  src/hsv_example_node.cpp
  src/HsvExample.cpp
)
add_dependencies(hsv_example_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(hsv_example_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

