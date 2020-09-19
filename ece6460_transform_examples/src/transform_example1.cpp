// ROS header
#include <ros/ros.h>

// Include the main header for the TF library
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{

  // Populate transform from global to vehicle
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(10, 3, 0));

  tf2::Quaternion q;
  q.setRPY(0, 0, 120 * M_PI / 180);
  transform.setRotation(q);

  tf2::Vector3 target_coordinates(25, 0, 0);

  tf2::Vector3 transformed_target = transform * target_coordinates;

  ROS_INFO("transformed coordinates: (%f, %f, %f)", transformed_target.x(), transformed_target.y(), transformed_target.z());

  return 0;
}