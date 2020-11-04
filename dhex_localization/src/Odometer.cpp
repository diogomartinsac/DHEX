#include <dhex_localization/Odometer.h>

Odometer::Odometer(
    const double &radius, const double &length, const geometry_msgs::Pose2D &pose) :
    m_radius(radius), m_length(length), m_pose(pose)
{
    // do nothing
}

void Odometer::updatePose(std_msgs::Float64 &left_velocity, 
                          std_msgs::Float64 &right_velocity,
                          double &dt)
{
    double velocity = (left_velocity.data + right_velocity.data) / 2;
    double omega = (left_velocity.data - right_velocity.data) / (2 * m_length);
    m_pose.x = m_pose.x + (velocity/omega) * (sin(m_pose.theta - omega) - sin(m_pose.theta));
    m_pose.y = m_pose.y + (velocity/omega) * (cos(m_pose.theta - omega) - cos(m_pose.theta));
    m_pose.theta = m_pose.theta + omega * (1/dt);
}

geometry_msgs::Pose2D Odometer::getPose()
{
    return m_pose;
}