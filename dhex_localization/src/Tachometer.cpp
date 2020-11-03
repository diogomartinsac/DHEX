#include<dhex_localization/Tachometer.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

Tachometer::Tachometer(
    const std::string &wheel_name, const std::string &base_name, const double &radius,
    const bool &right_wheel) : m_wheel_name(wheel_name), m_base_name(base_name), 
    m_radius(radius), m_right_wheel(right_wheel), m_first_measure_flag(true), 
    m_tf_listener(m_tf_buffer)
{   

}

Tachometer::~Tachometer(){
    m_tf_buffer.~Buffer();
    m_tf_listener.~TransformListener();
}

void Tachometer::computeWheelVelocity() 
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped last_published_tf;
    last_published_tf.header.frame_id = m_base_name;
    last_published_tf.child_frame_id = "previous_" + m_wheel_name;  
    last_published_tf.header.stamp = ros::Time::now();
    m_dt = ros::Time::now() - m_last_loop_time;
    m_last_loop_time = ros::Time::now();
    if (m_first_measure_flag) {
        geometry_msgs::TransformStamped base_to_wheel;
        try{
            base_to_wheel = m_tf_buffer.lookupTransform(m_base_name, m_wheel_name, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        tf2::Quaternion measured_quat;
        tf2::fromMsg(base_to_wheel.transform.rotation, measured_quat);
        last_published_tf.transform.rotation = tf2::toMsg(measured_quat);
        br.sendTransform(last_published_tf);
        m_first_measure_flag = false;
        return;
    } else {
        geometry_msgs::TransformStamped previous_to_wheel;
        try{
            previous_to_wheel = m_tf_buffer.lookupTransform("previous_" + m_wheel_name, m_wheel_name, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        geometry_msgs::TransformStamped base_to_wheel;
        try{
            base_to_wheel = m_tf_buffer.lookupTransform(m_base_name, m_wheel_name, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        tf2::Quaternion measured_quat, previous_quat, tf_quat;
        tf2::fromMsg(base_to_wheel.transform.rotation, measured_quat);
        tf2::fromMsg(previous_to_wheel.transform.rotation, previous_quat);
        std::cout<<"Measured Quat: "<<Tachometer::normilizeAngle(measured_quat.getAngle())<<std::endl;
        std::cout<<"Previous Quat: "<<Tachometer::normilizeAngle(previous_quat.getAngle())<<std::endl;
        std::cout<<"Period: "<<(double) m_dt.toSec()<<std::endl;
        if (m_right_wheel) 
            m_velocity.data = 0.7 * m_velocity.data + 
                - 0.3 * (Tachometer::normilizeAngle(measured_quat.getAngle())) * m_radius / m_dt.toSec();
        else 
            m_velocity.data = 0.7 * m_velocity.data + 
                0.3 * (measured_quat.getAngle() - m_last_angle) * m_radius / m_dt.toSec();
        double angle = Tachometer::normilizeAngle(Tachometer::normilizeAngle(measured_quat.getAngle()) +
            Tachometer::normilizeAngle(previous_quat.getAngle()));
        tf_quat.setRPY(0, 0, angle);
        std::cout<<"TF Quat: "<<Tachometer::normilizeAngle(tf_quat.getAngle())<<std::endl;
        last_published_tf.transform.rotation = tf2::toMsg(tf_quat);
        br.sendTransform(last_published_tf);
    }
}

double Tachometer::normilizeAngle(double angle)
{
    if (angle > M_PI) 
        return angle - 2 * M_PI;
    else return angle;
}

std_msgs::Float64 Tachometer::getWheelVelocity()
{
    return m_velocity;
}

std_msgs::Float64 Tachometer::getPeriod()
{
    std_msgs::Float64 dt;
    dt.data = m_dt.toSec();
    return dt;
}