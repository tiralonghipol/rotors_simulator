#include "acc_command_converter.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

AccCommandConverterNode::AccCommandConverterNode() 
  : receive_first_odom(false),
  receive_first_cmd(false)
{
  ros::NodeHandle nh;

  cmd_rate_thrust_sub_ = nh.subscribe("command/rate_thrust", 1,
                                     &AccCommandConverterNode::RateThrustCallback, this);

  cmd_roll_pitch_yawrate_thrust_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
      kDefaultCommandRollPitchYawrateThrustTopic, 1);

  ros::NodeHandle pnh("~");
  GetRosParameter(pnh, "use_vehicle_frame", true, &use_vehicle_frame); 
  GetRosParameter(pnh, "use_yaw_stabilize", true, &use_yaw_stabilize);
  GetRosParameter(pnh, "K_yaw", 1.8, &K_yaw_); 
  GetRosParameter(pnh, "yaw_rate_limit", M_PI/4, &yaw_rate_limit_); 
  GetRosParameter(pnh, "mass", 1.0, &mass); 
  ROS_WARN_STREAM("Mass:" << mass);
  //need to know current yaw angle of the robot if acc vector is expressed in world frame
  odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 1, &AccCommandConverterNode::OdometryCallback, this);
}

AccCommandConverterNode::~AccCommandConverterNode() { }

void AccCommandConverterNode::RateThrustCallback(
    const mav_msgs::RateThrustPtr& rate_thrust_msg) {
  if ((!use_vehicle_frame) && (!receive_first_odom))
  {
    return;
  }
  rate_thrust_cmd = *rate_thrust_msg;
  receive_first_cmd = true;
}


void AccCommandConverterNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("AccCommandConverter node got first odometry message.");
  if (!receive_first_odom)
  {
    frame_id = odometry_msg->header.frame_id; 
    receive_first_odom = true;
  }
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
  if (receive_first_cmd)
  {
    mav_msgs::RateThrust reference = rate_thrust_cmd;
    mav_msgs::RollPitchYawrateThrustPtr rpyrate_thrust_cmd(new mav_msgs::RollPitchYawrateThrust);
    Eigen::Vector3d current_rpy;
    odometry.getEulerAngles(&current_rpy);
    // double current_roll = current_rpy(0);
    // double current_pitch = current_rpy(1);
    double current_yaw = (use_vehicle_frame)?0.0:current_rpy(2);
    Eigen::Vector3d thrust_sp;
    thrust_sp << reference.thrust.x, reference.thrust.y, reference.thrust.z;
    thrust_sp = (thrust_sp + Eigen::Vector3d(0, 0, kGravity)) * mass;
    Eigen::Vector3d thrust_norm = thrust_sp.normalized();

    double cphi_stheta = thrust_norm(0) * cos(current_yaw) + thrust_norm(1) * sin(current_yaw);
    double sphi = thrust_norm(0) * sin(current_yaw) - thrust_norm(1) * cos(current_yaw);
    double cphi_ctheta = thrust_norm(2);
    if (cphi_ctheta != 0)
    {
      rpyrate_thrust_cmd->pitch = atan2(cphi_stheta, cphi_ctheta);
      rpyrate_thrust_cmd->roll = atan2(sphi, sqrt(cphi_stheta * cphi_stheta + cphi_ctheta * cphi_ctheta));
    }
    else
    {
      rpyrate_thrust_cmd->pitch = 0;
      rpyrate_thrust_cmd->roll = 0;
    }

    // YAW ctrl
    if (use_yaw_stabilize)
    {
      double yaw_error = 0 - current_rpy(2); // maintain zero-degree yaw angle

      if (std::abs(yaw_error) > M_PI)
      {
        if (yaw_error > 0.0)
        {
          while (yaw_error > M_PI)
          {
            yaw_error = yaw_error - 2.0 * M_PI;
          }
        }
        else
        {
          while (yaw_error < -M_PI)
          {
            yaw_error = yaw_error + 2.0 * M_PI;
          }
        }
      }

      double yaw_rate_cmd = K_yaw_ * yaw_error; // feed-forward yaw_rate cmd

      if (yaw_rate_cmd > yaw_rate_limit_)
      {
        yaw_rate_cmd = yaw_rate_limit_;
      }

      if (yaw_rate_cmd < -yaw_rate_limit_)
      {
        yaw_rate_cmd = -yaw_rate_limit_;
      }
      rpyrate_thrust_cmd->yaw_rate = yaw_rate_cmd; 
    }   
    else
    {
      rpyrate_thrust_cmd->yaw_rate = reference.angular_rates.z;
    }
    
    rpyrate_thrust_cmd->thrust.x = 0;
    rpyrate_thrust_cmd->thrust.y = 0;
    // total thrust is the norm of thrust vector
    rpyrate_thrust_cmd->thrust.z = thrust_sp.norm();
    
    // OR cross-product of thrust vector and current z_B axis
    //rpyrate_thrust_cmd->thrust.z = thrust_sp(0)*(cos(current_roll)*sin(current_pitch)*cos(current_yaw) + sin(current_roll)*sin(current_yaw))
    //                           + thrust_sp(1)*(cos(current_roll)*sin(current_pitch)*sin(current_yaw) - sin(current_roll)*cos(current_yaw))
    //                           + thrust_sp(2)*cos(current_roll)*cos(current_pitch);

    rpyrate_thrust_cmd->header.frame_id = frame_id;
    rpyrate_thrust_cmd->header.stamp = ros::Time::now();

    cmd_roll_pitch_yawrate_thrust_pub_.publish(rpyrate_thrust_cmd);   
  }
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "acc_command_converter");

  rotors_control::AccCommandConverterNode acc_command_converter_node;

  ros::spin();

  return 0;
}
