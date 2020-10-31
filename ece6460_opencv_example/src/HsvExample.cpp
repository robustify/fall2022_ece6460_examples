#include "HsvExample.hpp"

namespace ece6460_opencv_example {

HsvExample::HsvExample(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Read static input RGB image from PNG file and store in raw_rgb_img
  std::string filename = ros::package::getPath("ece6460_opencv_example") + "/hsv.png";
  cv::Mat raw_rgb_img = cv::imread(filename.c_str(), cv::IMREAD_COLOR);

  // Convert input image into the HSV colorspace
  cv::Mat raw_hsv_img;
  cv::cvtColor(raw_rgb_img, raw_hsv_img, cv::COLOR_BGR2HSV);

  // Split complete image into its three components
  std::vector<cv::Mat> split_img;
  cv::split(raw_hsv_img, split_img);

  // Extract each split channel into a dedicated OpenCV matrix class property for access in processing function
  hue_channel = split_img[0];
  sat_channel = split_img[1];
  val_channel = split_img[2];

  // This timer just refreshes the output displays to always reflect the current threshold settings
  refresh_timer_ = n.createTimer(ros::Duration(0.05), &HsvExample::timerCallback, this);

  // Set up dynamic reconfigure server to adjust threshold parameters
  srv_.setCallback(boost::bind(&HsvExample::reconfig, this, _1, _2));

  // Open windows to show the individual H, S, and V channels, and the thresholding output on each
  cv::namedWindow("H", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("S", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("V", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("H_thres", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("S_thres", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("V_thres", cv::WINDOW_AUTOSIZE);
}

void HsvExample::reconfig(HsvExampleConfig& config, uint32_t level)
{
  // Apply threshold to saturation channel
  // Mark all pixels with saturation above threshold as white (255)
  // Mark all pixels with saturation below threshold as black (0)
  cv::threshold(sat_channel, sat_thres, config.s_thres, 255, cv::THRESH_BINARY);

  // Apply threshold to value channel
  // Mark all pixels with value above threshold as white (255)
  // Mark all pixels with value below threshold as black (0)
  cv::threshold(val_channel, val_thres, config.v_thres, 255, cv::THRESH_BINARY);

  // Apply threshold to hue channel
  cv::Mat t1;
  cv::Mat t2;
  int h_pos_edge = config.h_center + config.h_width; // Upper edge of hue window
  int h_neg_edge = config.h_center - config.h_width; // Lower edge of hue window
  if (h_pos_edge > 180) {
    // Apply thresholds when upper edge overflows 180
    cv::threshold(hue_channel, t1, h_pos_edge - 180, 255, cv::THRESH_BINARY_INV);  
    cv::threshold(hue_channel, t2, config.h_center - config.h_width, 255, cv::THRESH_BINARY);  
    cv::bitwise_or(t1, t2, hue_thres);
  } else if (h_neg_edge < 0) {
    // Apply thresholds when lower edge underflows 0
    cv::threshold(hue_channel, t1, h_neg_edge + 180, 255, cv::THRESH_BINARY);  
    cv::threshold(hue_channel, t2, config.h_center + config.h_width, 255, cv::THRESH_BINARY_INV);  
    cv::bitwise_or(t1, t2, hue_thres);
  } else {
    // Apply thresholds when hue window is continuous
    cv::threshold(hue_channel, t1, config.h_center - config.h_width, 255, cv::THRESH_BINARY);
    cv::threshold(hue_channel, t2, config.h_center + config.h_width, 255, cv::THRESH_BINARY_INV);
    cv::bitwise_and(t1, t2, hue_thres);
  }
}

void HsvExample::timerCallback(const ros::TimerEvent& event)
{
  // Refresh display of individual channel images
  cv::imshow("H", hue_channel);
  cv::waitKey(1);
  cv::imshow("S", sat_channel);
  cv::waitKey(1);
  cv::imshow("V", val_channel);
  cv::waitKey(1);

  // Refresh display of threshold output images
  cv::imshow("H_thres", hue_thres);
  cv::waitKey(1);
  cv::imshow("S_thres", sat_thres);
  cv::waitKey(1);
  cv::imshow("V_thres", val_thres);
  cv::waitKey(1);
}

}
