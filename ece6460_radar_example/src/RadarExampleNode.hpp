// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>

// ROS message headers
#include <conti_radar_msgs/RadarObjectArray.h>
#include <avs_lecture_msgs/TrackedObjectArray.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ece6460_radar_example/RadarExampleConfig.h>

// Namespace matches ROS package name
namespace ece6460_radar_example {

  class RadarExampleNode {
    public:
      RadarExampleNode(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(RadarExampleConfig& config, uint32_t level);
      void recvRawObjects(const conti_radar_msgs::RadarObjectArrayConstPtr& msg);

      ros::Subscriber sub_raw_objects_;
      ros::Publisher pub_filtered_objects_;

      dynamic_reconfigure::Server<RadarExampleConfig> srv_;
      RadarExampleConfig cfg_;
  };

}
