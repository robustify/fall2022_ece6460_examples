// ROS and node class header file
#include <ros/ros.h>
#include "VectorToQuat.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "vector_to_quat");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  ece6460_vector_to_quat::VectorToQuat node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
