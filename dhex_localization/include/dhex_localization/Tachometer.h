#ifndef TACHOMETER_H
#define TACHOMETER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

class Tachometer 
{
private:
    bool m_right_wheel;
    std::string m_wheel_name;
    std::string m_base_name;
    double m_radius;
    std_msgs::Float64 m_velocity; 
    geometry_msgs::Twist m_tf_velocity;
    tf::TransformListener m_tf_listener;

public:
    Tachometer(const std::string &wheel_name, const std::string &base_name,
               const double &radius, const bool &right_wheel);

    ~Tachometer();

    void computeWheelVelocity();

    std_msgs::Float64 getWheelVelocity();
};

#endif