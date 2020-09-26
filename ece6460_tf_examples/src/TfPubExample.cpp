// Header file for the class
#include "TfPubExample.hpp"

// Namespace matches ROS package name
namespace ece6460_tf_examples {

  // Constructor with global and private node handle arguments
  TfPubExample::TfPubExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 100 Hz and call the 'timerCallback' method every trigger
    timer = n.createTimer(ros::Duration(0.01), &TfPubExample::timerCallback, this);

    // Initialize yaw angle to zero
    yaw = 0.0;
  }
  
  void TfPubExample::timerCallback(const ros::TimerEvent& event)
  {
    // Update yaw angle
    double yaw_rate = 1.0;
    yaw += 0.01 * yaw_rate;

    // Populate transform structure to publish to tf
    geometry_msgs::TransformStamped frame2_to_frame3;
    frame2_to_frame3.header.stamp = event.current_real; // Stamp at current time
    frame2_to_frame3.header.frame_id = "frame2"; // header.frame_id is the parent frame
    frame2_to_frame3.child_frame_id = "frame3"; // child_frame_id is the child frame

    // Fixed translation vector of (2, 1, 1)
    tf2::Vector3 translation_vector(2.0, 1.0, 1.0);

    // Convert tf2::Vector3 to geometry_msgs::Vector3
    tf2::convert(translation_vector, frame2_to_frame3.transform.translation);

    // Populate quaternion corresponding to current yaw angle
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    // Convert tf2::Quaternion to geometry_msgs::Quaternion
    tf2::convert(q, frame2_to_frame3.transform.rotation);

    // Publish the transform
    broadcaster.sendTransform(frame2_to_frame3);
  }

}
