#include<ros/ros.h>
#include<dhex_localization/Tachometer.h>
#include<dhex_localization/Odometer.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>

std_msgs::Float64 right_wheel_velocity, left_wheel_velocity;
geometry_msgs::Pose2D actual_pose;

class Parser 
{
public:
    double wheel_separation, wheel_radius;
    std::string base_name, wheel_name;
    bool right_wheel;

    Parser(ros::NodeHandle nh);
};

void odomCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    actual_pose.x = msg ->x;
    actual_pose.y = msg ->y;
    actual_pose.theta = msg ->theta;
}

void rightWheelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    right_wheel_velocity = (*msg);
}


void leftWheelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    left_wheel_velocity = (*msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");    
    ros::NodeHandle nh;
    ros::Rate loop_rate(250);    
    Parser parser(nh);
    ros::Publisher odom_publisher = nh.advertise<geometry_msgs::Pose2D>("/localization/odometry",10);
    ros::Subscriber odom = nh.subscribe("/dhex/odom", 1, odomCallback);
    ros::Subscriber right_wheel_sub = nh.subscribe("/localization/tachometer/right_wheel_velocity", 1, rightWheelCallback);
    ros::Subscriber left_wheel_sub = nh.subscribe("/localization/tachometer/left_wheel_velocity", 1, leftWheelCallback);
    geometry_msgs::Pose2D odometry;
    ros::Time timeNow(0), timePrev(0);
    timePrev = ros::Time::now();
    double dt;
    Odometer odometer(parser.wheel_radius, parser.wheel_separation, actual_pose);


    while(ros::ok())
    { 
        if ((ros::Time::now() - timePrev).toSec() >= 0.05){
            dt = (ros::Time::now() - timePrev).toSec();
            timePrev = ros::Time::now();     
            odometer.updatePose(left_wheel_velocity, right_wheel_velocity, dt);
            odometry = odometer.getPose();
            odom_publisher.publish(odometry);
        }
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}

Parser::Parser(ros::NodeHandle nh)
{
    nh.getParam("wheel_separation", this->wheel_separation);
    nh.getParam("wheel_radius", this->wheel_radius);
    nh.getParam("base_name", this->base_name);
    nh.getParam("wheel_name", this->wheel_name);
    nh.getParam("right_wheel", this->right_wheel);
}