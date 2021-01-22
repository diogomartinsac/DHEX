#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



def T_operation(data):
    
    teleOperation = Twist() # create the Twist Object
    teleOperation.linear.x = (0.2)*data.axes[1] #read the L button from joystick
    teleOperation.angular.z = data.axes[3] #read the R button from joystick
    dispatcher.publish(teleOperation) # .publish the datas from the joy to the cmd_vel topic



def start():

    global dispatcher
    
    dispatcher = rospy.Publisher('/dhex/cmd_vel', Twist,  queue_size=10)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, T_operation)
    # start the node
    rospy.init_node('Tele_Operation')
    rospy.spin()

if __name__ == '__main__':
    start()
    
