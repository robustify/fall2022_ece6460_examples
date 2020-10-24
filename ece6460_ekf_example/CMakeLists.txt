cmake_minimum_required(VERSION 3.0.2)
project(ece6460_ekf_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gps_common
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/EkfExample.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf_example
  src/ekf_example_node.cpp
  src/EkfExample.cpp  
)
add_dependencies(ekf_example
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(ekf_example
  ${catkin_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    src/${PROJECT_NAME}/rtk_gps_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
