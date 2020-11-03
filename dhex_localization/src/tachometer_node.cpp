#include<ros/ros.h>
#include<dhex_localization/Tachometer.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class Parser 
{
public:
    double wheel_radius;
    std::string base_name, wheel_name;
    bool right_wheel;

    Parser(ros::NodeHandle nh);
};


double wheel_speed_right;
double wheel_speed_left;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double vr = msg ->twist.twist.linear.x;
    double va = msg ->twist.twist.angular.z;
    wheel_speed_left = (vr - va * 0.09 / 2.0) / 0.0325;
    wheel_speed_right = (vr + va * 0.09 / 2.0) / 0.0325;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tachometer");    
    ros::NodeHandle nh;
    ros::Rate loop_rate(250);    
    Parser parser(nh);
    ros::Publisher velocity_publisher;
    ros::Publisher period_publisher;
    velocity_publisher = nh.advertise<std_msgs::Float64>("/wheel_velocity",10);
    period_publisher = nh.advertise<std_msgs::Float64>("/measurement_period",10);
    
    ros::Subscriber odom = nh.subscribe("/dhex/odom", 1, odomCallback);
    geometry_msgs::Twist objectTwist;
    tf::TransformListener listener;
    
    Tachometer tachometer(parser.wheel_name, parser.base_name,
                          parser.wheel_radius, parser.right_wheel);
    while(ros::ok())
    { 
        
        
        // left and right wheel speeds in rads/s

        try{
            listener.lookupTwist("/base_wheel", "/right_wheel", "base_wheel", tf::Point(), "right_wheel", ros::Time(0), ros::Duration(0.1), objectTwist);
        }
        catch (tf::LookupException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        std::cout<<" - Right Wheel speed: "<< objectTwist.angular.z * 0.0325 <<std::endl;
        std::cout<<"Right Wheel speed: "<< wheel_speed_right<<" Left wheel speed: " << wheel_speed_left<<std::endl;
        tachometer.computeWheelVelocity();  
        if (!std::isnan(tachometer.getWheelVelocity().data)) {
            velocity_publisher.publish(tachometer.getWheelVelocity());
            period_publisher.publish(tachometer.getPeriod());
        }
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}

Parser::Parser(ros::NodeHandle nh)
{
    nh.getParam("wheel_radius", this->wheel_radius);
    nh.getParam("base_name", this->base_name);
    nh.getParam("wheel_name", this->wheel_name);
    nh.getParam("right_wheel", this->right_wheel);
}