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
    ros::Time current_time = ros::Time::now();
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
    double dx = (velocity) * (cos(theta + omega));
    double dy = (velocity) * (sin(theta + omega));
    double dtheta = omega / dt;
    double v = sqrt ( dx*dx+dy*dy ) / dt;

    theta += dtheta;

    // if (theta  > M_PI) {
    //     theta = theta  - 2*M_PI;
    // } else if(theta  < M_PI){
    //     theta = theta + 2*M_PI; 
    // }

    quat.setRPY(0,0,theta);
    m_pose.pose.pose.position.x += dx;
    m_pose.pose.pose.position.y += dy;
    m_pose.pose.pose.position.z = 0;
    m_pose.pose.pose.orientation.x = quat.x();
    m_pose.pose.pose.orientation.y = quat.y();
    m_pose.pose.pose.orientation.z = quat.z();
    m_pose.pose.pose.orientation.w = quat.w();
    m_pose.twist.twist.angular.z = dtheta;
    m_pose.twist.twist.linear.x = v;
    m_pose.twist.twist.linear.y = 0;

    m_pose.header.stamp = current_time;

}

nav_msgs::Odometry Odometer::getPose()
{
    return m_pose;
}