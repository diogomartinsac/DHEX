#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

class Odometer 
{
private:
    geometry_msgs::Pose2D m_pose; 
    double m_radius, m_length;

public:
    Odometer(const double &radius, const double &length, const geometry_msgs::Pose2D &pose);
 
    void updatePose(std_msgs::Float64 &left_velocity, 
                    std_msgs::Float64 &right_velocity,
                    double &dt);

    geometry_msgs::Pose2D getPose();
};