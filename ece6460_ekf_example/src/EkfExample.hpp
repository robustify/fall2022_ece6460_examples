#pragma once

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <dynamic_reconfigure/server.h>
#include <ece6460_ekf_example/EkfExampleConfig.h>

#include <eigen3/Eigen/Dense>

namespace ece6460_ekf_example {

  typedef Eigen::Matrix<double, 5, 1> StateVector;
  typedef Eigen::Matrix<double, 5, 5> StateMatrix;

  class EkfExample {
    public:
      EkfExample(ros::NodeHandle n, ros::NodeHandle pn);

    private:
      void reconfig(EkfExampleConfig& config, uint32_t level);
      void timerCallback(const ros::TimerEvent& event);
      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
      void recvFix(const sensor_msgs::NavSatFixConstPtr& msg);

      // Methods to iterate the Kalman filter
      void updateFilterNone(const ros::Time& current_time);
      void updateFilterGPS(const ros::Time& current_time, const tf2::Vector3& position);
      void updateFilterTwist(const ros::Time& current_time, const geometry_msgs::Twist& twist);

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      ros::Subscriber sub_twist_;
      ros::Subscriber sub_fix_;
      ros::Publisher pub_odom_;
      ros::Timer timer_;

      dynamic_reconfigure::Server<EkfExampleConfig> srv_;
      EkfExampleConfig cfg_;

      tf2_ros::TransformBroadcaster broadcaster_;
      tf2::Vector3 ref_utm_vect_;

      // Estimate state, covariance, and current time stamp
      StateVector X_;
      StateMatrix P_;
      ros::Time estimate_stamp_;

      // Process noise covariance
      StateMatrix Q_;
  };

}

