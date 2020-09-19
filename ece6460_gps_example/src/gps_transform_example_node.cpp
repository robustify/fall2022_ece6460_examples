// ROS and node class header file
#include <ros/ros.h>
#include "GpsTransformExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "gps_transform_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_gps_example::GpsTransformExample node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
