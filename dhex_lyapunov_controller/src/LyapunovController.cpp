#include <dhex_lyapunov_controller/LyapunovController.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

LyapunovController::LyapunovController(std::string name, ros::NodeHandle param_nh) :
  as_(nh_, name, false),
  action_name_(name)
{
  as_.registerGoalCallback(boost::bind(&LyapunovController::executeCB, this));
  std::string control_topic, odom_topic;
  param_nh.getParam("sub_topic/odom", odom_topic);
  param_nh.getParam("pub_topic/control", control_topic);
  param_nh.getParam("distance_tolerance", distance_tolerance_);
  param_nh.getParam("angle_tolerance", angle_tolerance_);
  param_nh.getParam("k1", k1_);
  param_nh.getParam("k2", k2_);
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

double normalizeAngle(double angle)
{
  if (angle > M_PI) 
    return angle - 2 * M_PI;
  if (angle < -M_PI) 
    return angle + 2 * M_PI;
  else 
    return angle;
}

double LyapunovController::getThetaFromQuat(const geometry_msgs::Quaternion quat_msg)
{
  tf2::Quaternion quat;
  tf2::convert(quat_msg, quat);
  quat.normalize();
  return normalizeAngle(quat.getAngle());
}

double LyapunovController::calculateLyapunovOmega()
{
  const double azimuth = calculateAzimuth();
  const double current_delta = normalizeAngle(azimuth - current_pose_.theta);
  double goal_delta = normalizeAngle(azimuth - goal_.goal_pose.theta);
  double distance = calculateDistanceFromGoal();
  if (distance <= 10 * distance_tolerance_)
    distance = 10 * distance_tolerance_;
  return  (goal_.velocity/distance) * (k2_ * (current_delta - std::atan(-k1_ * goal_delta)) + 
          (1 + (k1_ / (1 + pow(k1_ * goal_delta, 2))) * sin(current_delta)));
}

double LyapunovController::calculateAzimuth()
{
  return atan2(feedback_.error_pose.y, feedback_.error_pose.x);
}

double LyapunovController::calculateDistanceFromGoal()
{
  return std::sqrt(pow(feedback_.error_pose.y,2) + pow(feedback_.error_pose.x,2));
}

void LyapunovController::calculateFeedback()
{
  // calculate feedback
  feedback_.error_pose.x = goal_.goal_pose.x - current_pose_.x;
  feedback_.error_pose.y = goal_.goal_pose.y - current_pose_.y;
  feedback_.error_pose.theta = normalizeAngle(goal_.goal_pose.theta - current_pose_.theta);
}

bool LyapunovController::hasReachedGoal()
{  
  return calculateDistanceFromGoal() <= std::fabs(distance_tolerance_) &&
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

    calculateFeedback();
    as_.publishFeedback(feedback_);

    // calculate and publish control output
    geometry_msgs::Twist control_output;
    control_output.linear.x = goal_.velocity;
    control_output.angular.z = calculateLyapunovOmega();
    control_pub_.publish(control_output);

    ros::spinOnce(); 
    r.sleep();
  }

  if(success)
  {
    geometry_msgs::Twist control_output;
    control_pub_.publish(control_output);
    result_.reached_goal = true;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
}