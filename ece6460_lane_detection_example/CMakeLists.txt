cmake_minimum_required(VERSION 3.0.2)
project(ece6460_lane_detection_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LaneDetection.cfg
)

catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detection_example
  src/lane_detection_example.cpp
  src/LaneDetection.cpp
)
add_dependencies(lane_detection_example
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(lane_detection_example
  ${catkin_LIBRARIES}
)

