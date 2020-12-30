#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/server/simple_action_server.h>
#include <dhex_lyapunov_controller/LyapunovControllerAction.h>
#include <dhex_lyapunov_controller/LyapunovControllerGoal.h>
#include <dhex_lyapunov_controller/LyapunovControllerResult.h>
#include <dhex_lyapunov_controller/LyapunovControllerFeedback.h>

class LyapunovController
{
protected:
  ros::NodeHandle nh_;
  double distance_tolerance_, angle_tolerance_, k1_, k2_;
  actionlib::SimpleActionServer<dhex_lyapunov_controller::LyapunovControllerAction> as_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  dhex_lyapunov_controller::LyapunovControllerGoal goal_;
  dhex_lyapunov_controller::LyapunovControllerFeedback feedback_;
  dhex_lyapunov_controller::LyapunovControllerResult result_;
  ros::Publisher control_pub_; 
  ros::Subscriber odom_sub_; 
  geometry_msgs::Pose2D current_pose_;

public:
  LyapunovController(std::string name, ros::NodeHandle param_nh);
  ~LyapunovController(void);

  double calculateAzimuth();
  double calculateDistanceFromGoal();
  double calculateLyapunovOmega();
  double getThetaFromQuat(const geometry_msgs::Quaternion quat_msg);
  bool hasReachedGoal();
  bool start();

  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
  void executeCB();
  void calculateFeedback();
};

#endif //LYAPUNOV_CONTROLLER_H