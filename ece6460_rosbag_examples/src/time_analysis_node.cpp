// ROS and node class header file
#include <ros/ros.h>
#include "TimeAnalysisNode.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "time_analysis_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_rosbag_examples::TimeAnalysisNode node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
