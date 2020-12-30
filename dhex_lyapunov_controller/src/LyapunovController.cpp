#include <dhex_lyapunov_controller/LyapunovController.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

LyapunovController::LyapunovController(std::string name, ros::NodeHandle param_nh) :
  as_(nh_, name, false),
  action_name_(name)
{
  kp_ = 4;
  as_.registerGoalCallback(boost::bind(&LyapunovController::executeCB, this));
  std::string control_topic, odom_topic;
  param_nh.getParam("sub_topic/odom", odom_topic);
  param_nh.getParam("pub_topic/control", control_topic);
  param_nh.getParam("distance_tolerance", distance_tolerance_);
  param_nh.getParam("angle_tolerance", angle_tolerance_);
  control_pub_ = nh_.advertise<geometry_msgs::Twist>(control_topic,1);
  odom_sub_ = nh_.subscribe(odom_topic, 1, &LyapunovController::odomCB, this);
  as_.start();
}

LyapunovController::~LyapunovController(void)
{
}

void LyapunovController::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;
  current_pose_.theta = getThetaFromQuat(msg->pose.pose.orientation);
}

double LyapunovController::getThetaFromQuat(const geometry_msgs::Quaternion quat_msg)
{
  tf2::Quaternion quat;
  tf2::convert(quat_msg, quat);
  quat.normalize();
  if (quat.getAngle() > M_PI) {
    return quat.getAngle() - 2*M_PI;
  } else {
    return quat.getAngle();
  }
}

double LyapunovController::calculateAzimuth()
{
  return atan2(feedback_.error_pose.y, feedback_.error_pose.x) - current_pose_.theta;
}

void LyapunovController::calculateFeedback()
{
  // calculate feedback
  feedback_.error_pose.x = goal_.goal_pose.x - current_pose_.x;
  feedback_.error_pose.y = goal_.goal_pose.y - current_pose_.y;
  feedback_.error_pose.theta = goal_.goal_pose.theta - current_pose_.theta;
}

bool LyapunovController::hasReachedGoal()
{  
  return std::fabs(feedback_.error_pose.x) <= std::fabs(distance_tolerance_) &&
         std::fabs(feedback_.error_pose.y) <= std::fabs(distance_tolerance_) &&
         std::fabs(feedback_.error_pose.theta) <= std::fabs(angle_tolerance_);
}

void LyapunovController::executeCB()
{
  goal_ = *(as_.acceptNewGoal());

  // helper variables
  ros::Rate r(100);
  bool success = true;
  calculateFeedback();

  // publish info to the console for the user
  ROS_INFO("%s: Executing, guiding vehicle to (x, y, theta) = (%f, %f, %f)",
           action_name_.c_str(), goal_.goal_pose.x, goal_.goal_pose.y, goal_.goal_pose.theta);
  // start executing the action
  while(!hasReachedGoal() && ros::ok())
  {
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      break;
    }

    std::cout<<"Pose in X: "<<current_pose_.x<<std::endl;

    calculateFeedback();
    as_.publishFeedback(feedback_);

    // calculate and publish control output
    geometry_msgs::Twist control_output;
    control_output.linear.x = goal_.velocity;
    control_output.angular.z = kp_ * calculateAzimuth();
    control_pub_.publish(control_output);

    ros::spinOnce(); 
    r.sleep();
  }

  if(success)
  {
    result_.reached_goal = true;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
}