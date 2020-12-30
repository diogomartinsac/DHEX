#include <stdlib.h>    
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dhex_lyapunov_controller/LyapunovControllerAction.h>
#include <dhex_lyapunov_controller/LyapunovControllerGoal.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pose_command");
  if (argc != 5)
  {
    ROS_ERROR("Must pass 4 arguments: -x_goal -y_goal -theta_goal -velocity");
    return 0;
  }

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<dhex_lyapunov_controller::LyapunovControllerAction> ac("lyapunov_controller", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  dhex_lyapunov_controller::LyapunovControllerGoal goal;
  goal.goal_pose.x = atof(argv[1]);
  goal.goal_pose.y = atof(argv[2]);
  goal.goal_pose.theta = atof(argv[3]);
  goal.velocity = atof(argv[4]);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}