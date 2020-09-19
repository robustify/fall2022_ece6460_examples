// ROS header
#include <ros/ros.h>

// Include the main header for the TF library
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{

  // Populate transform from global to vehicle
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(14, 14, 0));

  tf2::Quaternion q;
  q.setRPY(0, 0, 120 * M_PI / 180);
  transform.setRotation(q);

  tf2::Vector3 target_location(5, 27, 0);

  tf2::Vector3 target_location_in_vehicle = transform.inverse() * target_location;

  ROS_INFO("transformed coordinates: (%f, %f, %f)", target_location_in_vehicle.x(), target_location_in_vehicle.y(), target_location_in_vehicle.z());

  return 0;
}