#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import UInt16
#from am_driver.msg import SensorStatus
#from am_driver.msg import BatteryStatus
import signal


class SmartwatchControl(object):
#"""Class used to get the smartwatch information and control the hrp robot"""
    def init(self):
        #""" This function setup publishers and subcriber topics"""
        # Initial values
        self.inc_ratio = 0.1
        self.speed = np.array([0, ])
        self.update_rate = 10   # Hz

        # Setup publishers
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
        rospy.Subscriber('/wearami_acc',Pose,self.callback_continuos_control)

    def callback_continuos_control(self,data):
        # Here the acceleration data from /wearami_acc topic is obtained and processd

        # Get the acceleration data
        x = data.position.x
        y = data.position.y
        z = data.position.z
        # Update robot linear and angular velocity
        self.update(x,y,z)

	
    def run(self):
        #"""ROS execution"""
        try:
            self.init()
            #self.print_usage()
            r = rospy.Rate(self.update_rate) # Hz
            while not rospy.is_shutdown():
                r.sleep()
        except rospy.ROSInterruptException: # CHECK IF THIS IS WORKING
            self.update(0,0,0)
       #     pass
      #  finally:
      #      pass


    def update(self,x,y,z):
        # Here we transform the data of acceleration to angular and linear velocity to the robot
        if rospy.is_shutdown():
            return
        twist = Twist()
        twist.linear.x = x*0.05
        twist.angular.z = y*0.05
        self.pub_twist.publish(twist)


rospy.init_node('smartwatch_hrp_control')
control = SmartwatchControl()
control.run()
