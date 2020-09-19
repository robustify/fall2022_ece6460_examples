// Header file for the class
#include "GpsTransformExample.hpp"

// Namespace matches ROS package name
namespace ece6460_gps_example
{  
  // Constructor with global and private node handle arguments
  GpsTransformExample::GpsTransformExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // TODO: Set correct topic name
    sub_fix = n.subscribe("topic_name", 1, &GpsTransformExample::recvFix, this);


    double ref_lat = 42.6707444;
    double ref_lon = -83.2172277;

    std::string ref_utm_zone;
    // TODO: Convert reference coordinates to UTM
//     ROS_INFO("Reference UTM coordinates: (%f, %f) Zone: %s", ref_utm_x, ref_utm_y, ref_utm_zone.c_str());
  }

  void GpsTransformExample::recvFix(const gps_common::GPSFixConstPtr& msg)
  {
    // Convert current position to UTM
    double current_utm_x;
    double current_utm_y;
    std::string current_utm_zone;
    // TODO: Convert current coordinates to UTM
    ROS_INFO("Current UTM position: (%f, %f) Zone: %s", current_utm_x, current_utm_y, current_utm_zone.c_str());

    // TODO: Compute relative position vector in UTM
    double relative_pos_east;
    double relative_pos_north;
    ROS_INFO("Our current position is %f meters East and %f meters North of the reference point", relative_pos_east, relative_pos_north);

    // TODO: Convert heading from NED to ENU
    double enu_heading;
    double utm_heading; // TODO: account for convergence angle!

    // Construct transform from UTM to the vehicle frame
    tf2::Transform current_utm_transform;
    // TODO: Set translation component with UTM position
    // TODO: Set rotation component with UTM heading

    // TODO: Transform the UTM coordinates of the reference into the vehicle frame
    // ROS_INFO("The reference point is %f meters in front and %f meters to the left of our current heading\n", vehicle_frame_coords.x(), vehicle_frame_coords.y());
  }

}
