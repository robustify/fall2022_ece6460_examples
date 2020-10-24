// ROS and node class header file
#include <ros/ros.h>
#include "EkfExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "ekf_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  ece6460_ekf_example::EkfExample node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
