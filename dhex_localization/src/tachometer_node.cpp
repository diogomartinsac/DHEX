#include<ros/ros.h>
#include<dhex_localization/Tachometer.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class Parser 
{
public:
    double wheel_radius;
    std::string base_name, wheel_name;
    std::string pub_str_wheel_velocity;
    bool right_wheel;

    Parser(ros::NodeHandle nh_local, ros::NodeHandle nh_global);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tachometer");    
    ros::NodeHandle nh_local("~"), nh_global("/");    
    Parser parser(nh_local, nh_global);
    ros::Publisher velocity_publisher;
    Tachometer tachometer(parser.wheel_name, parser.base_name,
                          parser.wheel_radius, parser.right_wheel);
    velocity_publisher = 
        nh_global.advertise<std_msgs::Float64>(parser.pub_str_wheel_velocity,1);
    ros::Rate loop_rate(100);    

    while(ros::ok())
    { 
        tachometer.computeWheelVelocity();  
        if (!std::isnan(tachometer.getWheelVelocity().data)) {
            velocity_publisher.publish(tachometer.getWheelVelocity());
        }
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}

Parser::Parser(ros::NodeHandle nh_local, ros::NodeHandle nh_global)
{
    nh_global.getParam("wheel_radius", this->wheel_radius);
    nh_global.getParam("base_name", this->base_name);
    nh_local.getParam("wheel_name", this->wheel_name);
    nh_local.getParam("right_wheel", this->right_wheel);
    nh_local.getParam("pub_topic/wheel_velocity", this->pub_str_wheel_velocity);
}