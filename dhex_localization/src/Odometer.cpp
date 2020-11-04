#include <dhex_localization/Odometer.h>

Odometer::Odometer(){}

Odometer::Odometer(
    const double radius, const double length, const nav_msgs::Odometry pose) :
    m_radius(radius), m_length(length), m_pose(pose)
{
    tf2::Quaternion quat;
    std::cout<<"Quat INIT: "<<m_pose.pose.pose.orientation<<std::endl;
    tf2::convert(m_pose.pose.pose.orientation, quat);
    quat.normalize();
    double theta;
    if (quat.getAngle() > M_PI) {
        theta = quat.getAngle() - 2*M_PI;
    } else {
        theta = quat.getAngle();
    }
    std::cout<<"Theta INIT: "<<theta<<std::endl;
}

void Odometer::updatePose(std_msgs::Float64 &left_velocity, 
                          std_msgs::Float64 &right_velocity,
                          const double dt)
{
    tf2::Quaternion quat;
    std::cout<<"Quat b4: "<<m_pose.pose.pose.orientation<<std::endl;
    tf2::convert(m_pose.pose.pose.orientation, quat);
    quat.normalize();
    double theta = quat.getAngle();
    std::cout<<"Theta b4: "<<theta<<std::endl;
    double velocity = dt * (left_velocity.data + right_velocity.data) / 2.;
    std::cout<<"Radius: "<<m_radius<<"  Length: "<<m_length<<std::endl;

    double omega = (dt * (right_velocity.data - left_velocity.data) / ((m_length)));
    m_pose.pose.pose.position.x += (velocity) * (cos(theta + omega/2.));
    m_pose.pose.pose.position.y += (velocity) * (sin(theta + omega/2.));
    theta+=omega;
    if (theta  > 2 * M_PI) {
        theta = theta  - 2*M_PI;
    } else if (theta  < 0){
        theta = theta + 2*M_PI; 
    }
    std::cout<<"Omega: "<<omega<<std::endl;
    std::cout<<"Theta after: "<<theta<<std::endl;
    quat.setRPY(0,0,theta);
    quat.normalize();
    m_pose.pose.pose.orientation = tf2::toMsg(quat);
}

nav_msgs::Odometry Odometer::getPose()
{
    return m_pose;
}