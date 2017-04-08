#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus


class SmartwatchControl(object):
  
  def __init__(self):
    # Initial values
    self.inc_ratio = 0.1
    self.speed = np.array([0, ])
    self.update_rate = 10   # Hz

    # Setup publishers
    self.pub_twist = rospy.Publisher('/hrp_vel', Twist, queue_size=1)
    rospy.Subscriber('/wearami_acc',Pose,self.callback_continuos_control)
    self.run()


  def callback_continuos_control(self,data):
    x = data.position.x
    y = data.position.y
    z = data.position.z
    self.update(x,y,z)

	
  def run(self):
    """ROS execution"""
    try:
      #self.print_usage()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      pass

	

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

