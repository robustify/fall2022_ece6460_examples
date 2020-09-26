// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// TF library headers
#include <tf2_ros/transform_listener.h>

// Namespace matches ROS package name
namespace ece6460_tf_examples {

  class TfLookupExample {
    public:
      TfLookupExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      
      ros::Timer timer;
      tf2_ros::TransformListener listener;
      tf2_ros::Buffer buffer;

  };

}
