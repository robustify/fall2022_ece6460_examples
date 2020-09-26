// ROS and node class header file
#include <ros/ros.h>
#include "TfPubExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "tf_pub_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_tf_examples::TfPubExample node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
