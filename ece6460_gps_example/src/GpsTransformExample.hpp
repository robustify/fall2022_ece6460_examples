// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// Include the main header for the TF library
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS message containing raw position info
#include <gps_common/GPSFix.h>

// Header to convert to and from UTM
#include <gps_common/conversions.h>

// Namespace matches ROS package name
namespace ece6460_gps_example {

  class GpsTransformExample {
    public:
      GpsTransformExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void recvFix(const gps_common::GPSFixConstPtr& msg);

      ros::Subscriber sub_fix;
  };

}
