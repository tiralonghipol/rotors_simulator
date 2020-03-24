/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

LeePositionController::LeePositionController()
    : initialized_params_(false),
      controller_active_(false),
      in_pos_ctrl_mode_(true) {
  InitializeParameters();
}

LeePositionController::~LeePositionController() {}

void LeePositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  //velocity_error_integral_.setZero();
  initialized_params_ = true;
}

void LeePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
  //ROS_INFO_STREAM_THROTTLE(1, "acc:" << acceleration << ",angular acc:" << angular_acceleration << ",rotor" << *rotor_velocities); 
}

void LeePositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void LeePositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
  // if (!in_pos_ctrl_mode_)
  // {
  //   velocity_error_integral_.setZero();
  // }
  in_pos_ctrl_mode_ = true;
  ROS_INFO_STREAM("got pose cmd:" << command_trajectory_.position_W);
}

void LeePositionController::SetTwistCmd(
    const geometry_msgs::Twist& command_twist) {
  command_twist_ = command_twist;
  controller_active_ = true;
  in_pos_ctrl_mode_ = false;
}

void LeePositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

  static Eigen::Vector3d velocity_error_integral_(0.0, 0.0, 0.0);
  double current_time = ros::Time::now().toSec();
  static double last_time = current_time;
  double dt = current_time - last_time;
  last_time = current_time;
  if (dt > 0.5)
  {
    velocity_error_integral_.setZero();
  }

  if (in_pos_ctrl_mode_)
  {
    Eigen::Vector3d position_error;
    position_error = odometry_.position - command_trajectory_.position_W;

    // Transform velocity to world frame.
    const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
    Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
    Eigen::Vector3d velocity_error;
    velocity_error = velocity_W - command_trajectory_.velocity_W;

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

    *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
        + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
        - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
    // ROS_INFO_STREAM_THROTTLE(1, "pos ctrl mode:" << *acceleration);    
  }
  else // twist control
  {
    Eigen::Vector3d velocity_error;
    Eigen::Vector3d command_linear;
    // Transform velocity to world frame.
    const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
    double yaw = mav_msgs::yawFromQuaternion(odometry_.orientation);
    Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;    
    command_linear << command_twist_.linear.x, command_twist_.linear.y, command_twist_.linear.z;
    command_linear = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * command_linear;
    velocity_error = velocity_W - command_linear;
    velocity_error_integral_ = velocity_error_integral_ + velocity_error * dt;
    // limit integral terms
    double integration_limit = 1.0;
    for (int i = 0; i < 3; i++)
    {
      if (velocity_error_integral_(i) > integration_limit)
      {
        velocity_error_integral_(i) = integration_limit;
      }
      else if (velocity_error_integral_(i) < -integration_limit)
      {
        velocity_error_integral_(i) = -integration_limit;
      }
    }

    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

    *acceleration = (velocity_error_integral_.cwiseProduct(controller_parameters_.position_gain_)
        + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)) / vehicle_parameters_.mass_
        - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
    // ROS_INFO_STREAM_THROTTLE(1, "twist ctrl mode:" << *acceleration);     
  }
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  double yaw;
  if (in_pos_ctrl_mode_)
  {
    yaw = command_trajectory_.getYaw();
  }
  else // twist control mode
  {
    //Eigen::Vector3d odom_euler;
    //mav_msgs::getEulerAnglesFromQuaternion(odometry_.orientation, &odom_euler);
    //yaw = odom_euler(2);
    yaw = mav_msgs::yawFromQuaternion(odometry_.orientation);
  }
  b1_des << cos(yaw), sin(yaw), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  if (in_pos_ctrl_mode_)
  {
    angular_rate_des[2] = command_trajectory_.getYawRate();
  }
  else
  {
    angular_rate_des[2] = command_twist_.angular.z;
  }

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
}
}
