// ROS and node class header file
#include <ros/ros.h>
#include "UlcExampleNode.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "ulc_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_ulc_example::UlcExampleNode node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
