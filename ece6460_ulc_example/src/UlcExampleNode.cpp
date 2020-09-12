// Header file for the class
#include "UlcExampleNode.hpp"

// Namespace matches ROS package name
namespace ece6460_ulc_example {

  // Constructor with global and private node handle arguments
  UlcExampleNode::UlcExampleNode(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 50 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(0.02), &UlcExampleNode::timerCallback, this);

    // Advertise UlcCmd topic
    pub_ulc_cmd_ = n.advertise<dataspeed_ulc_msgs::UlcCmd>("/vehicle/ulc_cmd", 1);

    // Initialize dynamic reconfigure server and bind its callback
    srv_.setCallback(boost::bind(&UlcExampleNode::reconfig, this, _1, _2));
  }

  void UlcExampleNode::timerCallback(const ros::TimerEvent& event)
  {
    // Declare message structure variable
    dataspeed_ulc_msgs::UlcCmd ulc_cmd_msg;

    // Populate message
    ulc_cmd_msg.enable_pedals = cfg_.enable_pedals;
    ulc_cmd_msg.enable_steering = cfg_.enable_steering;
    ulc_cmd_msg.enable_shifting = true;
    ulc_cmd_msg.linear_velocity = cfg_.linear_velocity;
    ulc_cmd_msg.yaw_command = cfg_.yaw_command;
    ulc_cmd_msg.steering_mode = cfg_.steering_mode;

    // Publish message
    pub_ulc_cmd_.publish(ulc_cmd_msg);
  }

  void UlcExampleNode::reconfig(UlcExampleConfig& config, uint32_t level)
  {
    // Copy latest GUI parameters into class member for access elsewhere
    cfg_ = config;
  }
}
