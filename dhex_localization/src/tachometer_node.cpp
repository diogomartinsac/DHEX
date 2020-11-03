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
    bool right_wheel;

    Parser(ros::NodeHandle &nh);
    void deleteParameters(ros::NodeHandle &nh);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tachometer");    
    ros::NodeHandle nh;    
    Parser parser(nh);
    ros::Publisher velocity_publisher;
    Tachometer tachometer(parser.wheel_name, parser.base_name,
                          parser.wheel_radius, parser.right_wheel);
    velocity_publisher = 
        nh.advertise<std_msgs::Float64>("localization/tachometer/" + parser.wheel_name +"_velocity",10);
    parser.deleteParameters(nh);
    ros::Rate loop_rate(10);    

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

Parser::Parser(ros::NodeHandle &nh)
{
    nh.getParam("wheel_radius", this->wheel_radius);
    nh.getParam("base_name", this->base_name);
    nh.getParam("wheel_name", this->wheel_name);
    nh.getParam("right_wheel", this->right_wheel);
}

void Parser::deleteParameters(ros::NodeHandle &nh)
{    
    nh.deleteParam("wheel_radius");
    nh.deleteParam("base_name");
    nh.deleteParam("wheel_name");
    nh.deleteParam("right_wheel");
}