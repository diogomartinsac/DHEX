#ifndef TACHOMETER_H
#define TACHOMETER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

class Tachometer 
{
private:
    bool m_first_measure_flag;
    bool m_right_wheel;
    double m_last_angle;
    std::string m_wheel_name;
    std::string m_base_name;
    double m_radius;
    std_msgs::Float64 m_velocity; 
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;
    ros::Time m_last_loop_time;
    ros::Duration m_dt;

public:
    Tachometer(const std::string &wheel_name, const std::string &base_name,
               const double &radius, const bool &right_wheel);

    ~Tachometer();

    void computeWheelVelocity();

    std_msgs::Float64 getWheelVelocity();

    std_msgs::Float64 getPeriod();

    static double normilizeAngle(const double angle);
};

#endif