#include <ros/ros.h>
#include "ClusterExample.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cluster_example_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ece6460_pointcloud_example::ClusterExample node(n, pn);

  ros::spin();
}
