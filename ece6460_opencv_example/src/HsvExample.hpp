#pragma once

// ROS header
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <dynamic_reconfigure/server.h>
#include <ece6460_opencv_example/HsvExampleConfig.h>
#include <ros/package.h>

namespace ece6460_opencv_example {

  class HsvExample
  {
    public:
      HsvExample(ros::NodeHandle n, ros::NodeHandle pn);
      
    private:
      void reconfig(HsvExampleConfig& config, uint32_t level);
      void timerCallback(const ros::TimerEvent& event);
      
      ros::Timer refresh_timer_;
      
      dynamic_reconfigure::Server<HsvExampleConfig> srv_;
      
      // Input image channels
      cv::Mat hue_channel;
      cv::Mat sat_channel;
      cv::Mat val_channel;

      // Threshold output images
      cv::Mat hue_thres;
      cv::Mat sat_thres;
      cv::Mat val_thres;
  };
}
