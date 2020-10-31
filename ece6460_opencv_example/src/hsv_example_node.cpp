// ROS and node class header file
#include <ros/ros.h>
#include "HsvExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "hsv_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_opencv_example::HsvExample node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
