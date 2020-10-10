#include <ros/ros.h>
#include "PassthroughExample.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "passthrough_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ece6460_pointcloud_example::PassthroughExample node(n, pn);

  ros::spin();
}
