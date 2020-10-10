#include "PassthroughExample.hpp"
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

namespace ece6460_pointcloud_example
{

  PassthroughExample::PassthroughExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    srv_.setCallback(boost::bind(&PassthroughExample::reconfig, this, _1, _2));

    sub_cloud_ = n.subscribe("points", 10, &PassthroughExample::recvCloud, this);
    pub_filtered_cloud_ = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
  }

  void PassthroughExample::reconfig(PointCloudExampleConfig& config, uint32_t level)
  {
    cfg_ = config;
  }

  void PassthroughExample::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Instantiate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Copy ROS message data into PCL cloud
    pcl::fromROSMsg(*msg, *input_cloud);

    // Instantiate passthrough filter and array of filtered point indices
    pcl::IndicesPtr roi_indices(new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud(input_cloud);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName("x");
    pass.setFilterLimits(cfg_.x_min, cfg_.x_max);
    pass.filter(*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices(roi_indices);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(cfg_.y_min, cfg_.y_max);
    pass.filter(*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices(roi_indices);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(cfg_.z_min, cfg_.z_max);
    pass.filter(*filtered_cloud);

    // Copy filtered cloud data into a ROS message
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);

    // Publish output point cloud
    pub_filtered_cloud_.publish(output_msg);
   
  }

}
