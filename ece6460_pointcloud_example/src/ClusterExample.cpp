#include "ClusterExample.hpp"

namespace ece6460_pointcloud_example
{

  ClusterExample::ClusterExample(ros::NodeHandle& n, ros::NodeHandle& pn) :
    kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    srv_.setCallback(boost::bind(&ClusterExample::reconfig, this, _1, _2));

    sub_cloud_ = n.subscribe("points", 10, &ClusterExample::recvCloud, this);
    pub_filtered_cloud_ = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
    pub_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("objects", 1);
    pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);
  }

  void ClusterExample::reconfig(PointCloudExampleConfig& config, uint32_t level)
  {
    cfg_ = config;
  }

  void ClusterExample::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Instantiate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Copy ROS message data into PCL cloud
    pcl::fromROSMsg(*msg, *input_cloud);

    filterRawCloud(input_cloud, filtered_cloud);

    // Compute normal vectors for the incoming point cloud
    pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    kd_tree_->setInputCloud(filtered_cloud);
    normal_estimator.setSearchMethod(kd_tree_);
    normal_estimator.setInputCloud(filtered_cloud);
    normal_estimator.setKSearch(cfg_.num_normal_neighbors);
    normal_estimator.compute(*cloud_normals);

    // TODO: Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    for (int i = 0; i < cloud_normals->points.size(); i++) {
      non_vertical_normals.indices.push_back(i);
    }

    // Copy non-vertical normals into a separate cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*filtered_cloud, non_vertical_normals, *no_ground_cloud);

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(filtered_cloud->header);
    normals_.poses.clear();
    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = filtered_cloud->points[non_vertical_normals.indices[i]].x;
      p.position.y = filtered_cloud->points[non_vertical_normals.indices[i]].y;
      p.position.z = filtered_cloud->points[non_vertical_normals.indices[i]].z;

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      tf2::Quaternion q;
      tf2::Matrix3x3 rot_mat;
      if (fabs(nz) > 0.5) {
        rot_mat[0] = tf2::Vector3(nx, ny, nz);
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
        rot_mat[1].normalize();
        rot_mat[2] = rot_mat[0].cross(rot_mat[1]);
      } else {
        rot_mat[0] = tf2::Vector3(nx, ny, nz);
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
        rot_mat[1].normalize();
        rot_mat[2] = rot_mat[0].cross(rot_mat[1]);
      }
      rot_mat.getRotation(q);
      tf2::convert(q.inverse(), p.orientation);
      normals_.poses.push_back(p);
    }
    // Publish normal vectors
    pub_normals_.publish(normals_);

    // Run Euclidean clustering and extract set of indices arrays
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cluster_tol);
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    kd_tree_->setInputCloud(no_ground_cloud);
    ec.setSearchMethod(kd_tree_);
    ec.setInputCloud(no_ground_cloud);
    ec.extract(cluster_indices);

    // Use indices arrays to separate point cloud into individual clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    for (auto indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*no_ground_cloud, indices, *cluster);
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      cluster_clouds.push_back(cluster);
    }

    // Merge individual cluster clouds into a ROS PointCloud2 message for Rviz debugging
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& cluster : cluster_clouds) {
      merged_cloud->points.insert(merged_cloud->points.begin(), cluster->points.begin(), cluster->points.end());
    }
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = true;

    // Compute bounding boxes around clusters
    pcl::PointXYZ min_point, max_point;

    bboxes_.header = pcl_conversions::fromPCL(filtered_cloud->header);
    bboxes_.objects.clear();
    for (auto& cluster : cluster_clouds) {
      pcl::getMinMax3D(*cluster, min_point, max_point);
      avs_lecture_msgs::TrackedObject box;
      box.header = bboxes_.header;
      box.bounding_box_scale.x = max_point.x - min_point.x;
      box.bounding_box_scale.y = max_point.y - min_point.y;
      box.bounding_box_scale.z = max_point.z - min_point.z;
      box.pose.position.x = 0.5 * (max_point.x + min_point.x);
      box.pose.position.y = 0.5 * (max_point.y + min_point.y);
      box.pose.position.z = 0.5 * (max_point.z + min_point.z);
      box.pose.orientation.w = 1.0;
      bboxes_.objects.push_back(box);
    }
    // Publish bounding boxes
    pub_bboxes_.publish(bboxes_);

    // Copy filtered cloud data into a ROS message
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*merged_cloud, output_msg);
    output_msg.header = pcl_conversions::fromPCL(filtered_cloud->header);

    // Publish output point cloud
    pub_filtered_cloud_.publish(output_msg);
  }

  void ClusterExample::filterRawCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    // Instantiate passthrough filter and array of filtered point indices
    pcl::IndicesPtr roi_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    pass.setInputCloud (cloud_in);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (cfg_.x_min, cfg_.x_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Y range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (cfg_.y_min, cfg_.y_max);
    pass.filter (*roi_indices);

    // Ask passthrough filter to extract points in a given Z range
    pass.setIndices (roi_indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (cfg_.z_min, cfg_.z_max);
    pass.filter (*cloud_out);

    // Run through a voxel grid filter to downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(cloud_out);
    downsample.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
    downsample.filter(*cloud_out);
  }

}