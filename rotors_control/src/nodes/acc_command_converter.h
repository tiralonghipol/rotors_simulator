#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/RateThrust.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/roll_pitch_yawrate_thrust_controller.h"

namespace rotors_control {

class AccCommandConverterNode {
 public:
  AccCommandConverterNode();
  ~AccCommandConverterNode();

 private:
  static constexpr double kGravity = 9.8066;

  bool use_vehicle_frame;
  bool receive_first_odom;
  bool receive_first_cmd;
  bool receive_goal;
  bool use_yaw_stabilize;
  mav_msgs::EigenOdometry odometry;
  mav_msgs::RateThrust rate_thrust_cmd;
  mav_msgs::EigenOdometry goal_odometry;
  std::string frame_id;
  double K_yaw_;
  double yaw_rate_limit_;
  double Kp_x_, Kp_y_, Kp_z_;
 
  double mass;

  // subscribers
  ros::Subscriber cmd_rate_thrust_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber goal_pose_sub_;

  ros::Publisher cmd_roll_pitch_yawrate_thrust_pub_;
  ros::Publisher state_action_pub_;

  void RateThrustCallback(const mav_msgs::RateThrustPtr& rate_thrust_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void GoalPoseCallback(const geometry_msgs::Pose& goal_msg);

  void convertGoal2WorldFrame(const geometry_msgs::Pose& goal, const mav_msgs::EigenOdometry& robot_odom, mav_msgs::EigenOdometry *goal_in_world);

  void convertGoal2VehicleFrame(const mav_msgs::EigenOdometry& goal_odom, const mav_msgs::EigenOdometry& robot_odom,
    nav_msgs::Odometry *goal_in_vehicle_frame);  
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H
