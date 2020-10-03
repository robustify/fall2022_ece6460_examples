// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// ROS message headers
#include <geometry_msgs/TwistStamped.h>

// Namespace matches ROS package name
namespace ece6460_rosbag_examples {

  class TimeAnalysisNode {
    public:
      TimeAnalysisNode(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
      
      ros::Timer timer_;
      ros::Subscriber sub_twist_;

  };

}
