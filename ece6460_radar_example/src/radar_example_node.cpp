// ROS and node class header file
#include <ros/ros.h>
#include "RadarExampleNode.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "radar_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_radar_example::RadarExampleNode node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
