// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// ROS message headers
#include <dataspeed_ulc_msgs/UlcCmd.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ece6460_ulc_example/UlcExampleConfig.h>

// Namespace matches ROS package name
namespace ece6460_ulc_example {

  class UlcExampleNode {
    public:
      UlcExampleNode(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      void reconfig(UlcExampleConfig& config, uint32_t level);

      ros::Timer timer_;
      ros::Publisher pub_ulc_cmd_;

      dynamic_reconfigure::Server<UlcExampleConfig> srv_;
      UlcExampleConfig cfg_;

  };

}
