// ROS and node class header file
#include <ros/ros.h>
#include "ParamLoader.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "param_loader_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_launch_example::ParamLoader node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
