
#include<dhex_localization/Tachometer.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

Tachometer::Tachometer(
    const std::string &wheel_name, const std::string &base_name, const double &radius,
    const bool &right_wheel) : m_wheel_name(wheel_name), m_base_name(base_name), 
    m_radius(radius), m_right_wheel(right_wheel)
{   

}

Tachometer::~Tachometer()
{
    m_tf_listener.~TransformListener();
}

void Tachometer::computeWheelVelocity() 
{
    try{
        m_tf_listener.lookupTwist("/" + m_base_name, "/" + m_wheel_name, m_base_name, 
            tf::Point(), m_wheel_name, ros::Time(0), ros::Duration(0.1), m_tf_velocity);
    }
    catch (tf::LookupException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    if (m_right_wheel)
        m_velocity.data = - m_tf_velocity.angular.z * m_radius;
    else  m_velocity.data = m_tf_velocity.angular.z * m_radius;
}

std_msgs::Float64 Tachometer::getWheelVelocity()
{
    return m_velocity;
}