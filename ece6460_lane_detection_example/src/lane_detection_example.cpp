#include <ros/ros.h>
#include "LaneDetection.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ece6460_lane_detection_example::LaneDetection node(n, pn);
  
  ros::spin();
}