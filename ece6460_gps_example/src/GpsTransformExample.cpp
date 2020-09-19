// Header file for the class
#include "GpsTransformExample.hpp"

// Namespace matches ROS package name
namespace ece6460_gps_example
{  
  // Constructor with global and private node handle arguments
  GpsTransformExample::GpsTransformExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sub_fix = n.subscribe("/vehicle/perfect_gps/enhanced_fix", 1, &GpsTransformExample::recvFix, this);

    double ref_lat = 42.6707444;
    double ref_lon = -83.2172277;

    std::string ref_utm_zone;
    gps_common::LLtoUTM(ref_lat, ref_lon, ref_utm_y, ref_utm_x, ref_utm_zone);
    ROS_INFO("Reference UTM coordinates: (%f, %f) Zone: %s", ref_utm_x, ref_utm_y, ref_utm_zone.c_str());
  }

  void GpsTransformExample::recvFix(const gps_common::GPSFixConstPtr& msg)
  {
    // Convert current position to UTM
    double current_utm_x;
    double current_utm_y;
    std::string current_utm_zone;
    gps_common::LLtoUTM(msg->latitude, msg->longitude, current_utm_y, current_utm_x, current_utm_zone);
    ROS_INFO("Current UTM position: (%f, %f) Zone: %s", current_utm_x, current_utm_y, current_utm_zone.c_str());

    // Relative position vector in UTM
    double relative_pos_east = current_utm_x - ref_utm_x;
    double relative_pos_north = current_utm_y - ref_utm_y;
    ROS_INFO("Our current position is %f meters East and %f meters North of the reference point", relative_pos_east, relative_pos_north);

    // TODO: Convert heading from NED to ENU
    double enu_heading = (M_PI / 2) - (M_PI / 180 * msg->track);
    double utm_heading = enu_heading; // TODO: account for convergence angle!

    // Construct transform from UTM to the vehicle frame
    tf2::Transform current_utm_transform;
    // Translation component with UTM position
    current_utm_transform.setOrigin(tf2::Vector3(current_utm_x, current_utm_y, 0));
    // Rotation component with UTM heading
    tf2::Quaternion q;
    q.setRPY(0, 0, utm_heading);
    current_utm_transform.setRotation(q);

    // Transform the UTM coordinates of the reference into the vehicle frame
    tf2::Vector3 vehicle_frame_coords = current_utm_transform.inverse() * tf2::Vector3(ref_utm_x, ref_utm_y, 0);
    ROS_INFO("The reference point is %f meters in front and %f meters to the left of our current heading\n", vehicle_frame_coords.x(), vehicle_frame_coords.y());
  }

}
