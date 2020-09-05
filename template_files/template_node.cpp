// ROS and node class header file
#include <ros/ros.h>
#include "Template.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "template_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  new_package::Template node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
