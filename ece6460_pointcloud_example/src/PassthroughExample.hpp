#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <ece6460_pointcloud_example/PointCloudExampleConfig.h>

namespace ece6460_pointcloud_example
{

  class PassthroughExample
  {
    public:
      PassthroughExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(PointCloudExampleConfig& config, uint32_t level);
      void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

      ros::Publisher pub_filtered_cloud_;
      ros::Subscriber sub_cloud_;

      dynamic_reconfigure::Server<PointCloudExampleConfig> srv_;
      PointCloudExampleConfig cfg_;
  };

}
