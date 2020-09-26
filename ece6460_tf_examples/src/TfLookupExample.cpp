// Header file for the class
#include "TfLookupExample.hpp"

// Namespace matches ROS package name
namespace ece6460_tf_examples {

  // Constructor with global and private node handle arguments
  TfLookupExample::TfLookupExample(ros::NodeHandle& n, ros::NodeHandle& pn) :
    listener(buffer) // This is very important!
  {
    // Create a timer to run at 1 Hz and call the 'timerCallback' method every trigger
    timer = n.createTimer(ros::Duration(1.0), &TfLookupExample::timerCallback, this);
  }

  void TfLookupExample::timerCallback(const ros::TimerEvent& event)
  {
    // Declare a variable to hold the transform information
    geometry_msgs::TransformStamped transform;

    try {
      // Attempt to look up the transform from frame1 to frame3
      transform = buffer.lookupTransform("frame1", "frame3", ros::Time(0));
    } catch (tf2::TransformException& ex) {
      // If exception is thrown, display the error message and return
      ROS_WARN_STREAM(ex.what());
      return;
    }

    // Show results on terminal
    ROS_INFO_STREAM("Successfully looked up transform from " << transform.header.frame_id
                                                   << " to " << transform.child_frame_id);
    ROS_INFO_STREAM("Translation: (" << transform.transform.translation.x << ", "
                                     << transform.transform.translation.y << ", "
                                     << transform.transform.translation.z << ")");
    ROS_INFO_STREAM("Orientation: (" << transform.transform.rotation.w << ", "
                                     << transform.transform.rotation.x << ", "
                                     << transform.transform.rotation.y << ", "
                                     << transform.transform.rotation.z << ")\n");
  }
}
