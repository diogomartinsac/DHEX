#include<ros/ros.h>
#include<dhex_localization/Tachometer.h>
#include <std_msgs/Float64.h>

class Parser 
{
public:
    double wheel_radius;
    std::string base_name, wheel_name;
    bool right_wheel;

    Parser(ros::NodeHandle nh);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tachometer");    
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);    
    Parser parser(nh);
    ros::Publisher velocity_publisher;
    ros::Publisher period_publisher;
    velocity_publisher = nh.advertise<std_msgs::Float64>("/wheel_velocity",10);
    period_publisher = nh.advertise<std_msgs::Float64>("/measurement_period",10);
    Tachometer tachometer(parser.wheel_name, parser.base_name,
                          parser.wheel_radius, parser.right_wheel);
    while(ros::ok())
    { 
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