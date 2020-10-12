// Header file for the class
#include "VectorToQuat.hpp"

// Namespace matches ROS package name
namespace ece6460_vector_to_quat {

  // Constructor with global and private node handle arguments
  VectorToQuat::VectorToQuat(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 20 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(0.05), &VectorToQuat::timerCallback, this);

    // Initialize the dynamic reconfigure server to provide inputs to the example
    srv_.setCallback(boost::bind(&VectorToQuat::reconfig, this, _1, _2));

    // Advertise Rviz marker array topic
    pub_marker_array_ = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
  }

  void VectorToQuat::timerCallback(const ros::TimerEvent& event)
  {
    // Extract components into separate variables for code readability
    double nx = vect.x();
    double ny = vect.y();
    double nz = vect.z();

    // Construct rotation matrix to align frame transform with the normal vector
    tf2::Matrix3x3 rot_mat;
    // First basis vector is the vector we want to align
    rot_mat[0] = tf2::Vector3(nx, ny, nz);
    if (std::abs(nz) < 0.9) {
      // Vector is not close to vertical --> use x and y components to create orthogonal vector
      rot_mat[1] = tf2::Vector3(-ny, nx, 0);
    } else {
      // Vector is close to vertical --> use y and z components to make orthogonal vector
      rot_mat[1] = tf2::Vector3(0, -nz, ny);
    }
    // Normalize the generated orthogonal vector, because it is not necessarily unit length
    rot_mat[1].normalize();
    // Cross product produces the third basis vector of the rotation matrix
    rot_mat[2] = rot_mat[0].cross(rot_mat[1]);

    // Extract equivalent quaternion representation for the transform
    // rot_mat.transpose() is used because the basis vectors should be loaded
    // into the columns of the matrix, but the indexing in the above commands set the rows
    //   of the matrix instead of the columns.
    tf2::Quaternion q;
    rot_mat.transpose().getRotation(q);

    // Generate and publish Rviz marker message to visualize output
    publishMarkers(rot_mat, event.current_real);
  }

  void VectorToQuat::publishMarkers(const tf2::Matrix3x3& rot_mat, const ros::Time& stamp)
  {
    visualization_msgs::MarkerArray marker_array_msg;
    visualization_msgs::Marker m;
    // Common properties
    m.header.frame_id = "base_frame";
    m.header.stamp = stamp;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::ARROW;
    m.scale.x = 0.1;
    m.scale.y = 0.2;
    m.pose.orientation.w = 1;
    m.points.resize(2);
    m.points[0].x = 0.0;
    m.points[0].y = 0.0;
    m.points[0].z = 0.0;

    // Input vector
    m.id = 0;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.points[1].x = rot_mat[0].x();
    m.points[1].y = rot_mat[0].y();
    m.points[1].z = rot_mat[0].z();
    marker_array_msg.markers.push_back(m);

    // Y axis vector
    m.id++;
    m.color.a = 0.2;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.points[1].x = rot_mat[1].x();
    m.points[1].y = rot_mat[1].y();
    m.points[1].z = rot_mat[1].z();
    marker_array_msg.markers.push_back(m);

    // Z axis vector
    m.id++;
    m.color.a = 0.2;
    m.color.r = 0.0;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.points[1].x = rot_mat[2].x();
    m.points[1].y = rot_mat[2].y();
    m.points[1].z = rot_mat[2].z();
    marker_array_msg.markers.push_back(m);

    pub_marker_array_.publish(marker_array_msg);
  }

  void VectorToQuat::reconfig(VectorToQuatConfig& config, uint32_t level)
  {
    vect = tf2::Vector3(config.x, config.y, config.z);
    vect.normalize();
    config.x = vect.x();
    config.y = vect.y();
    config.z = vect.z();
  }

}
