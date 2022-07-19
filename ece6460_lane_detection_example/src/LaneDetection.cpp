#include "LaneDetection.hpp"

#define DEBUG 1

namespace ece6460_lane_detection_example
{

LaneDetection::LaneDetection(ros::NodeHandle n, ros::NodeHandle pn) :
  listener_(buffer_)
{
  sub_cam_info_ = n.subscribe("camera_info", 1, &LaneDetection::recvCameraInfo, this);
  sub_image_ = n.subscribe("image_rect_color", 1, &LaneDetection::recvImage, this);
  pub_markers_ = n.advertise<visualization_msgs::MarkerArray>("projected_lines", 1);
  pub_cloud_ = n.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);

  srv_.setCallback(boost::bind(&LaneDetection::reconfig, this, _1, _2));
  looked_up_camera_transform_ = false;

  pn.param("camera_name", camera_name_, std::string("front_camera"));

#if DEBUG
  cv::namedWindow("Binary", cv::WINDOW_NORMAL);
#endif
}

// This function is called whenever a new image is received from either
// the live running camera driver, a bag file recording, or the simulated
// image coming from Gazebo
void LaneDetection::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Do nothing until the coordinate transform from footprint to camera is valid,
  // because otherwise there is no point in detecting a lane!
  if (!looked_up_camera_transform_) {
    try {
      geometry_msgs::TransformStamped transform = buffer_.lookupTransform("base_footprint", camera_name_, msg->header.stamp);
      tf2::convert(transform.transform, camera_transform_);
      looked_up_camera_transform_ = true; // Once the lookup is successful, there is no need to keep doing the lookup
                                          // because the transform is constant
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    }
    return;
  }

  // Convert ROS image message into an OpenCV Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_img = cv_ptr->image;
  cv::Mat bin_img;
  segmentImage(raw_img, bin_img);
#if DEBUG
  cv::imshow("Binary", bin_img);
  cv::waitKey(1);
#endif

  // Create pinhole camera model instance and load
  // its parameters from the camera info
  // generated using the checkerboard calibration program
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);

  // Project points from 2D pixel coordinates into 3D where it intersects
  // the ground plane, and put them in a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Project every fourth row of the image to save some computational resources
  for (int i = 0; i < bin_img.rows; i += 4) {
    for (int j = 0; j < bin_img.cols; j++) {
      if (bin_img.at<uint8_t>(i, j) == 255) {
        // We found a white pixel corresponding to a lane marking. Project to ground
        // and add to point cloud
        geometry_msgs::Point proj_p = projectPoint(model, cv::Point(j, i));
        pcl::PointXYZ p;
        p.x = proj_p.x;
        p.y = proj_p.y;
        p.z = proj_p.z;
        bin_cloud->points.push_back(p);
      }
    }
  }
  
  // Publish point cloud to visualize in Rviz
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*bin_cloud, cloud_msg);
  cloud_msg.header.frame_id = "base_footprint";
  cloud_msg.header.stamp = msg->header.stamp;
  pub_cloud_.publish(cloud_msg);

  // TODO: Use Euclidean clustering to group dashed lines together
  // and separate each distinct line into separate point clouds

  // TODO: Construct polynomial curve fits to each cluster cloud

  // TODO: Construct and publish Rviz marker output to visualize curve fit

}

void LaneDetection::segmentImage(const cv::Mat& raw_img, cv::Mat& bin_img)
{
  // Convert to HSV colorspace
  cv::Mat raw_hsv;
  cv::cvtColor(raw_img, raw_hsv, CV_BGR2HSV);

  // Split HSV image into separate single-channel images for H, S, and V
  // and store each in dedicated variables
  std::vector<cv::Mat> split_img;
  cv::split(raw_hsv, split_img);
  cv::Mat hue_img = split_img[0];
  cv::Mat sat_img = split_img[1];
  cv::Mat val_img = split_img[2];

  // Detect white lane marking pixels in the image
  cv::Mat white_bin_img = cv::Mat::zeros(raw_hsv.size(), CV_8U);
  detectWhite(sat_img, val_img, white_bin_img);

  // Detect yellow lane marking pixels in the image
  cv::Mat yellow_bin_img = cv::Mat::zeros(raw_hsv.size(), CV_8U);
  detectYellow(hue_img, sat_img, yellow_bin_img);

  // Combine yellow and white detection with bitwise OR,
  // also applying a mask to ignore the hood of the car
  // that is in frame
  cv::Mat mask = cv::Mat::ones(white_bin_img.size(), CV_8U);
  mask(cv::Rect(0, cfg_.mask_height, mask.cols, mask.rows - cfg_.mask_height - 1)) = 0;
  cv::bitwise_or(white_bin_img, yellow_bin_img, bin_img, mask);

  // Apply Canny edge detection to greatly reduce the number
  // of detected pixels
  cv::Canny(bin_img, bin_img, 2, 4);
}

void LaneDetection::detectWhite(const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img)
{
  // TODO: Apply threshold to generate a binary value image. White lines have
  // higher value than the road

  // TODO: Apply inverse threshold to generate a binary saturation image. We want
  // to throw out high saturation pixels because white has very low saturation

  // TODO: Apply bitwise AND to make sure only pixels that satisfy both value and saturation
  // thresholds make it out. Store result in ouput (white_bin_img)
}

void LaneDetection::detectYellow(const cv::Mat& hue_img, const cv::Mat& sat_img, cv::Mat& yellow_bin_img)
{
  // TODO: Threshold hue

  // TODO: Threshold saturation

  // TODO: Bitwise AND to make sure only pixels that satisfy both hue and saturation
  // thresholds are detected. Store result in output (yellow_bin_img)
}

// Project 2D pixel point 'p' into vehicle's frame and return as 3D point
geometry_msgs::Point LaneDetection::projectPoint(const image_geometry::PinholeCameraModel& model, const cv::Point2d& p)
{
  // TODO: Convert the input pixel coordinates into a 3d ray, where x and y are projected to the point where z is equal to 1.0
  
  // TODO: Represent camera frame ray in footprint frame

  // TODO: Using the concept of similar triangles, scale the unit vector such that the end is on the ground plane.

  // TODO: Then add camera position offset to obtain the final coordinates in footprint frame

  // TODO: Fill output point with the result of the projection
  geometry_msgs::Point point;
  return point;
}

void LaneDetection::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_ = *msg;
}

void LaneDetection::reconfig(LaneDetectionConfig& config, uint32_t level)
{
  cfg_ = config;
}

}