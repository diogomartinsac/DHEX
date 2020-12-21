#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist


class VelocityFilter():
  
  def __init__(self, alpha):
    rospy.init_node('velocity_filter_exploration', anonymous=True)

    self.pub = rospy.Publisher('/cmd_vel_exploration', Twist, queue_size=1)

    self.sub = rospy.Subscriber("/move_base_exploration/cmd_vel", Twist, self.callback)

    self.alpha = alpha
    self.cmd_vel_filtered = Twist()
    

  def callback(self, cmd_vel_new):
    self.cmd_vel_filtered.linear.x = self.cmd_vel_filtered.linear.x*(1-self.alpha) + self.alpha*cmd_vel_new.linear.x
    self.cmd_vel_filtered.angular.z = self.cmd_vel_filtered.angular.z*(1-self.alpha) + self.alpha*cmd_vel_new.angular.z
    self.pub.publish(self.cmd_vel_filtered)

if __name__ == "__main__":
  rospy.loginfo("Velocity Filter Enabled")
  move = VelocityFilter(0.2)
  while not rospy.is_shutdown():
    rospy.spin()   
