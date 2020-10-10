#pragma once

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <ece6460_pointcloud_example/PointCloudExampleConfig.h>

// Message headers
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <avs_lecture_msgs/TrackedObjectArray.h>

// PCL processing headers
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

namespace ece6460_pointcloud_example
{

  class ClusterExample
  {
    public:
      ClusterExample(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void reconfig(PointCloudExampleConfig& config, uint32_t level);
      void recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

      void filterRawCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);

      ros::Publisher pub_filtered_cloud_;
      ros::Publisher pub_normals_;
      ros::Publisher pub_bboxes_;
      ros::Subscriber sub_cloud_;

      dynamic_reconfigure::Server<PointCloudExampleConfig> srv_;
      PointCloudExampleConfig cfg_;

      // Output messages
      geometry_msgs::PoseArray normals_;
      avs_lecture_msgs::TrackedObjectArray bboxes_;

      // KD search tree object for use by PCL functions
      pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;
  };

}