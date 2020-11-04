#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Odometer 
{
private:
    nav_msgs::Odometry m_pose; 
    double m_radius, m_length;

public:
    Odometer();

    Odometer(const double radius, const double length, const nav_msgs::Odometry pose);
 
    void updatePose(std_msgs::Float64 &left_velocity, 
                    std_msgs::Float64 &right_velocity,
                    const double dt);

    nav_msgs::Odometry getPose();
};