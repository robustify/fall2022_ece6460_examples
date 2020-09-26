// ROS and node class header file
#include <ros/ros.h>
#include "TfLookupExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "tf_lookup_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_tf_examples::TfLookupExample node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
