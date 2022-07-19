// Header file for the class
#include "RadarExampleNode.hpp"

// Namespace matches ROS package name
namespace ece6460_radar_example {

  // Constructor with global and private node handle arguments
  RadarExampleNode::RadarExampleNode(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Set up publishers and subscribers
    sub_raw_objects_ = n.subscribe("radar_objects", 1, &RadarExampleNode::recvRawObjects, this);
    pub_filtered_objects_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("filtered_radar_objects", 1);

    // Initialize dynamic reconfigure server and bind its callback
    srv_.setCallback(boost::bind(&RadarExampleNode::reconfig, this, _1, _2));
  }

  void RadarExampleNode::recvRawObjects(const conti_radar_msgs::RadarObjectArrayConstPtr& msg)
  {
    avs_lecture_msgs::TrackedObjectArray detected_object_msg;

    // Do nothing if input array of objects is empty
    if (msg->objects.size() == 0) {
      return;
    }

    detected_object_msg.header.stamp = ros::Time::now();
    detected_object_msg.header.frame_id = msg->objects[0].header.frame_id;

    for (auto obj : msg->objects) {
      // TODO: Add vehicle speed to the current object's x velocity and skip it
      // if the resulting velocity is small

      // Use RADAR data to build a TrackedObject structure
      avs_lecture_msgs::TrackedObject detected_object;
      detected_object.header = obj.header;
      detected_object.pose.position.x = obj.position.x;
      detected_object.pose.position.y = obj.position.y;
      detected_object.pose.position.z = obj.position.z;
      detected_object.pose.orientation.w = cos(0.5 * obj.orientation_angle);
      detected_object.pose.orientation.z = sin(0.5 * obj.orientation_angle);
      detected_object.bounding_box_scale = obj.dimensions;

      // Sometimes the RADAR can't detect the size of an object, in which
      // case the scale will be reported as zero. These can still be useful though,
      // so we put a fake size of 1 meter so it shows up in Rviz.
      if (detected_object.bounding_box_scale.x < 1e-3) {
        detected_object.bounding_box_scale.x = 1.0;
      }
      if (detected_object.bounding_box_scale.y < 1e-3) {
        detected_object.bounding_box_scale.y = 1.0;
      }
      if (detected_object.bounding_box_scale.z < 1e-3) {
        detected_object.bounding_box_scale.z = 1.0;
      }
      detected_object.velocity.linear = obj.velocity;
      detected_object.id = obj.object_id;
      detected_object_msg.objects.push_back(detected_object);
    }

    pub_filtered_objects_.publish(detected_object_msg);
  }

  void RadarExampleNode::reconfig(RadarExampleConfig& config, uint32_t level)
  {
    // Copy latest GUI parameters into class member for access elsewhere
    cfg_ = config;
  }
}
