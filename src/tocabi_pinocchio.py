#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import pinocchio
from sys import argv
from os.path import dirname, join, abspath

qmsg = JointState()
start = False

def callback(data):
      global qmsg
      global start 
      qmsg = data
      start = True

def talker():
      rospy.init_node('tocabi_pinocchio', anonymous = False)
      rospy.Subscriber("/tocabi/jointstates", JointState, callback)
      pub = rospy.Publisher('/tocabi/pinocchio',Float64MultiArray,queue_size=10)
      msg = Float64MultiArray()
      msg.data = [0 for i in range(33*6)]
      rate = rospy.Rate(1000) #10hz
      model = pinocchio.buildModelFromUrdf("/home/jhk/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi_sim.urdf")
      data = model.createData()
      global start

      while not rospy.is_shutdown():
            if start:
                  q = pinocchio.randomConfiguration(model)
                  global qmsg
                  for i in range(0, 33):
                        q[i] = qmsg.position[i]
                  CMM = pinocchio.computeCentroidalMap(model, data, q)
                  for i in range(0, 6):
                        for j in range(0, 33):
                              msg.data[33*(i)+j] = CMM[i][j] 
                  
                  pub.publish(msg)
                  rate.sleep()

if __name__=='__main__':
      try:
            talker()
      except rospy.ROSInterruptException:
            pass