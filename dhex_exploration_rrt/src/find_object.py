#!/usr/bin/env python 

# libraries:
import cv2
import math
import time
import rospy
import numpy as np
import os
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID 


class Robot:

  def __init__(self):
     # focal length
    self.focalLength = 937.8194580078125
    # bridge object to convert cv2 to ros and ros to cv2
    self.bridge = CvBridge()
    # timer var
    self.start = time.time()
    # create a camera node
    rospy.init_node('find_object', anonymous=True)
    # controllers
    self.xControl = Controller(5, -5, 0.01, 0, 0)
    self.yawControl = Controller(2, -2, 0.005, 0, 0)
    # image publisher object
    self.image_pub = rospy.Publisher('/dhex_camera/mission', Image, queue_size=1)
    # get camera info
    rospy.Subscriber("/dhex_camera/camera_info", CameraInfo, self.callback_camera_info)
    # cmd_vel
    self.velocity_ajustment = rospy.Publisher("/cmd_vel_object_adjustment", Twist, queue_size=1)
    # move to goal 
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple_object/goal", PoseStamped, queue_size=1)
    self.msg_move_to_goal = PoseStamped()
    self.flag = True
    self.camera_info = CameraInfo()
    self.distance_filtered = 0
    self.x_move_base_filtered = 0
    self.y_move_base_filtered = 0
    self.timer_flag = time.time()
    self.counter = 0
    self.controller_flag = False
    self.error_distance = 999
    self.distance_controller = 0

    self.cancel_move_base = rospy.Publisher("/move_base/cancel", GoalID, queue_size = 1)



  def callback(self, data):
    # setup timer and font
    timer = int(time.time() - self.start)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # convert img to cv2
    cv2_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

    ### COLOR DETECTION ###
    # define range of yellow color
    yellowLower = (10, 110, 110)
    yellowUpper = (32, 255, 255)

    # hsv color-space convert
    hsv = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)

    # erosion and dilation for noise removal
    maskYellow = cv2.erode(maskYellow, None, iterations=2)
    maskYellow = cv2.dilate(maskYellow, None, iterations=2) 

    # find contours of image (cv2.CHAIN_APPROX_SIMPLE is for memory saves)
    cnt_yellow = cv2.findContours(maskYellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    ### CIRCLE DETECTION ###    
    contours_poly = []
    centers = []
    radius = []     
    
    # approximate contours to polygons + get bounding rects and circles
    for index, obj_cnt in enumerate(cnt_yellow):
      contours_poly.append(cv2.approxPolyDP(obj_cnt, 0.009 * cv2.arcLength(obj_cnt, True), True))
      aux1, aux2 = cv2.minEnclosingCircle(contours_poly[index])
      centers.append(aux1)
      radius.append(aux2)
      ## if the camera find the sphere ##
      if(len(contours_poly[index]) > 10):
        # draw a circle in sphere and put a warning message
        cv2.circle(cv2_frame, (int(centers[index][0]), int(centers[index][1])), int(radius[index]), (0, 0, 255), 5) 
        cv2.putText(cv2_frame, 'GOAL ON SIGHT!', (20, 130), font, 2, (0, 0, 255), 5)
        # print info on terminal
        print('CONTROL INFO :')
        print('radius: ' + str(radius[0]))
        print('center x position: ' + str(centers[0][0]))
        #print('linear vel: ' + str(linear_vel))                
        #print('angular vel: ' + str(angular_vel))
        self.goal_move_base(centers[0][0], radius[0])
        print('##################################')
        if self.controller_flag:
          control_input = Twist()
          control_input.linear.x =  self.xControl.calculate(1, 160, radius[0])
          control_input.angular.z = self.yawControl.calculate(1, self.camera_info.width/2, centers[0][0])
          self.velocity_ajustment.publish(control_input)
      else:
        self.controller_flag = False  
        self.xControl.reset()
        self.yawControl.reset()
        self.error_distance = 999
        self.counter = 0
    # merge timer info to frame
    if self.error_distance < 20:
      cv2.putText(cv2_frame, str(self.error_distance), (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv2_frame, str(self.counter), (10, 700), font, 2, (50, 255, 50), 6)

    # convert img to ros and pub image in a topic
    ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
    self.image_pub.publish(ros_frame)


  def callback_camera_info(self, data):
    self.camera_info = data

  def listener(self):
    # subscribe to a topic
    rospy.Subscriber('/dhex_camera/image_raw', Image, self.callback)  
    # simply keeps python from exiting until this node is stopped
    rospy.spin()

  
  def goal_move_base(self, center_ball, radius):
    distance = (1 * self.focalLength) / (radius * 2)
    y_move_base = -(center_ball - self.camera_info.width/2) / (radius*2) 
    x_move_base = math.sqrt(distance**2 - y_move_base**2)
    self.distance_filtered = 0.5*self.distance_filtered + 0.5*distance
    self.x_move_base_filtered = 0.5*self.x_move_base_filtered + 0.5*x_move_base
    self.y_move_base_filtered = 0.5*self.y_move_base_filtered + 0.5*y_move_base
    self.msg_move_to_goal.pose.position.x = x_move_base
    self.msg_move_to_goal.pose.position.y = y_move_base
    self.msg_move_to_goal.pose.orientation.w = 1
    self.msg_move_to_goal.header.frame_id = 'base_link'
    if self.flag and abs(distance- self.distance_filtered) < 7 and self.distance_filtered > 4:
      self.pub_move_to_goal.publish(self.msg_move_to_goal)
      self.flag = False
      self.timer_flag = time.time()
    if time.time() - self.timer_flag > 5:
      self.flag = True      
    if abs(distance- self.distance_filtered) < 1 and self.distance_filtered < 4:
      self.cancel_move_base.publish()
      self.controller_flag = True

    print('Euclidean distance to goal: ' + str(distance))
    print('Filtered euclidean distance to goal: ' + str(self.distance_filtered))
    print('Error: ' + str(distance - self.distance_filtered))
    print('X relative to '+ self.camera_info.header.frame_id + ': '  + str(x_move_base))
    print('Filtered X relative to '+ self.camera_info.header.frame_id + ': '  + str(self.x_move_base_filtered))
    print('Y relative to '+ self.camera_info.header.frame_id + ': '  + str(y_move_base))
    print('Filtered Y relative to '+ self.camera_info.header.frame_id + ': '  + str(self.y_move_base_filtered))
    self.counter += 1
    self.error_distance = distance - self.distance_filtered
    self.distance_controller = distance
    print(str(self.counter))



class Controller:
  sat_max = 0
  sat_min = 0
  kp = 0
  ki = 0
  kd = 0
  error_integral = 0 
  error_prev = 0 

  def __init__ (self, sat_max, sat_min, kp, ki, kd):
    self.sat_max = sat_max 
    self.sat_min = sat_min 
    self.kp = kp 
    self.ki = ki 
    self.kd = kd 
    
  def calculate(self, time, setpoint, process):
    # set the error
    self.error = setpoint - process
    self.error_integral =+ self.error
    # calculate the output
    control_output = self.kp*self.error + self.ki*(self.error_integral)*time + self.kd*(self.error - self.error_prev)/time    
    # using saturation max and min in control_output 
    if (control_output > self.sat_max):
      control_output = self.sat_max
    elif (control_output < self.sat_min):
      control_output = self.sat_min
    # set error_prev for kd   
    self.error_prev = self.error   
    return control_output  
  def reset(self):
    self.error = 0
    self.error_integral = 0  
    self.error_prev = 0
  
# main function
if __name__	== '__main__':
  try:
    mission = Robot()  
    mission.listener()  
  except rospy.ROSInterruptException:
    pass			