#include<ros/ros.h>
#include<dhex_localization/Tachometer.h>
#include<dhex_localization/Odometer.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "ros/time.h"

std_msgs::Float64 right_wheel_velocity, left_wheel_velocity;
nav_msgs::Odometry actual_pose;
bool right = false;
bool left = false;

class Parser 
{
public:
    double wheel_separation, wheel_radius;
    std::string base_name, wheel_name;
    std::string sub_str_init_pose, sub_str_left_tachometer, sub_str_right_tachometer;
    std::string pub_str_odom;
    bool right_wheel;

    Parser(ros::NodeHandle nh_local, ros::NodeHandle nh_global);
};

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    actual_pose = (*msg);
}

void rightWheelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    right_wheel_velocity = (*msg);
    right = true;
}

void leftWheelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    left_wheel_velocity = (*msg);
    left = true;
}

int main(int argc, char **argv)
{
    bool initial_pose_captured = false;
    ros::init(argc, argv, "odometry");    
    ros::NodeHandle nh_local("~"), nh_global("/");
    double rate = 5;
    ros::Rate loop_rate(rate);    
    Parser parser(nh_local, nh_global);
    ros::Publisher odom_publisher = nh_global.advertise<nav_msgs::Odometry>(parser.pub_str_odom,1);
    ros::Subscriber odom = nh_global.subscribe(parser.sub_str_init_pose, 1, odomCallback);
    ros::Subscriber right_wheel_sub = nh_global.subscribe(parser.sub_str_right_tachometer, 1, rightWheelCallback);
    ros::Subscriber left_wheel_sub = nh_global.subscribe(parser.sub_str_left_tachometer, 1, leftWheelCallback);
    nav_msgs::Odometry odometry;
    Odometer odometer;
    ros::Time timeNow(0), timePrev(0);
    timePrev = ros::Time::now();
    double dt;

    while(ros::ok())
    { 
        if (!initial_pose_captured) {
            tf2::Quaternion quat;
            quat[0] = actual_pose.pose.pose.orientation.x;
            quat[1] = actual_pose.pose.pose.orientation.y;
            quat[2] = actual_pose.pose.pose.orientation.z;
            quat[3] = actual_pose.pose.pose.orientation.w;
            quat.normalize();
            if (!std::isnan(quat.getAngle())) {
                initial_pose_captured = true;
                odometer = Odometer(parser.wheel_radius, parser.wheel_separation, actual_pose);
            }
        } else {
                if (right && left) {
                    odometer.updatePose(left_wheel_velocity, right_wheel_velocity, 1/rate);
                    odometry = odometer.getPose();
                    odometry.header.frame_id = "odom";
                    odometry.child_frame_id = "base_link";
                    odometry.header.stamp = ros::Time::now();
                    odom_publisher.publish(odometry);
                    right = false;
                    left = false;
                }
        }
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}

Parser::Parser(ros::NodeHandle nh_local, ros::NodeHandle nh_global)
{
    nh_global.getParam("wheel_separation", this->wheel_separation);
    nh_global.getParam("wheel_radius", this->wheel_radius);
    nh_local.getParam("pub_topic/odom", this->pub_str_odom);
    nh_local.getParam("sub_topic/initial_pose_topic", this->sub_str_init_pose);
    nh_local.getParam("sub_topic/left_wheel_tachometer", this->sub_str_left_tachometer);
    nh_local.getParam("sub_topic/right_wheel_tachometer", this->sub_str_right_tachometer);
}