// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// Namespace matches ROS package name
namespace ece6460_launch_example {

  class ParamLoader {
    public:
      ParamLoader(ros::NodeHandle& n, ros::NodeHandle& pn);

  };

}
