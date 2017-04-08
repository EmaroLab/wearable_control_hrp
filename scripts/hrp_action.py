#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import UInt16


class hrp_set_velocity(object):
  
  def __init__(self):
    """ This function setup publishers and subcriber topics
    """
    # Initial values
    self.update_rate = 10   # Hz

    # Setup publishers
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)

    # Here we receive the velocity that we want to set to the robot
    rospy.Subscriber('/hrp_vel',Twist,self.callback_continuos_control)
    self.run()


  def callback_continuos_control(self,data):
    """ ROS Callback of the /hrp_vel topic, it update the velocity to the hrp robot
        param data: Twist to be given to the robot 
    """
    if rospy.is_shutdown():
      return
    self.pub_twist.publish(data)

	
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


rospy.init_node('set_hrp_velocity')
robot_action = hrp_set_velocity()

