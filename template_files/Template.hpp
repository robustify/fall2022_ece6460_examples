// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// Namespace matches ROS package name
namespace new_package {

  class Template {
    public:
      Template(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      // Node-specific stuff here
  };

}
