#include <dhex_localization/Odometer.h>

Odometer::Odometer(
    const double &radius, const double &length, const nav_msgs::Odometry &pose) :
    m_radius(radius), m_length(length), m_pose(pose)
{
    // do nothing
}

void Odometer::updatePose(std_msgs::Float64 &left_velocity, 
                          std_msgs::Float64 &right_velocity,
                          double &dt)
{
    tf2::Quaternion quat;
    tf2::fromMsg(m_pose.pose.pose.orientation, quat);
    double theta;
    if (quat.getAngle() > M_PI) {
        theta = quat.getAngle() - 2*M_PI;
    } else {
        theta = quat.getAngle();
    }
    double velocity = (left_velocity.data + right_velocity.data) / 2;
    double omega = (left_velocity.data - right_velocity.data) / (2 * m_length);
    m_pose.pose.pose.position.x = m_pose.pose.pose.position.x + (velocity) * (cos(theta + omega));
    m_pose.pose.pose.position.y = m_pose.pose.pose.position.y + (velocity) * (sin(theta + omega));
    theta = theta + omega * (1/dt);
    if (theta  > M_PI) {
        theta = theta  - 2*M_PI;
    } else if(theta  < M_PI){
        theta = theta + 2*M_PI; 
    }
    quat.setRPY(0,0,theta);
    m_pose.pose.pose.orientation = tf2::toMsg(quat);
}

nav_msgs::Odometry Odometer::getPose()
{
    return m_pose;
}