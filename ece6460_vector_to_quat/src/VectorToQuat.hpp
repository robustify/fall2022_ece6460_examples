// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS message headers
#include <visualization_msgs/MarkerArray.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ece6460_vector_to_quat/VectorToQuatConfig.h>

// Namespace matches ROS package name
namespace ece6460_vector_to_quat {

  class VectorToQuat {
    public:
      VectorToQuat(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      void reconfig(VectorToQuatConfig& config, uint32_t level);

      void publishMarkers(const tf2::Matrix3x3& rot_mat, const ros::Time& stamp);

      ros::Timer timer_;
      ros::Publisher pub_marker_array_;

      dynamic_reconfigure::Server<VectorToQuatConfig> srv_;
      VectorToQuatConfig cfg_;

      tf2::Vector3 vect;
  };

}
