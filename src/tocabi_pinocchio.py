#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tocabi_controller.msg import model

import pinocchio
import numpy as np
from sys import argv
from os.path import dirname, join, abspath

qmsg = JointState()
modelmsg = model()
start = False

def callback(data):
      global qmsg
      global start 
      qmsg = data
      start = True

def talker():
      rospy.init_node("tocabi_pinocchio", anonymous = False)
      rospy.Subscriber("/tocabi/pinocchio/jointstates", JointState, callback)
      global model
      pub = rospy.Publisher("/tocabi/pinocchio", model, queue_size=1)
      modelmsg.CMM = [0 for i in range(33*6)]
      modelmsg.COR = [0 for i in range(33*33)]
      modelmsg.M = [0 for i in range(33*33)]
      modelmsg.g = [0 for i in range(33)]

      rate = rospy.Rate(1000) #10hz
      model = pinocchio.buildModelFromUrdf("/home/jhk/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi.urdf",pinocchio.JointModelFreeFlyer())      
      data = model.createData()
      global start
      q = pinocchio.randomConfiguration(model)
      qdot = pinocchio.utils.zero(model.nv)
      qdot_t = pinocchio.utils.zero(model.nv)
      qddot_t = pinocchio.utils.zero(model.nv)

      while not rospy.is_shutdown():
             if start:
                  global qmsg
                  for i in range(0, 39):
                        q[i] = qmsg.position[i]
                        
                  for j in range(0, 38):
                        qdot[j] = qmsg.velocity[j]
                        qdot_t[j] = 0.0
                        qddot_t[j] = 0.0

                  CMM = pinocchio.computeCentroidalMap(model, data, q)
                  pinocchio.crba(model, data, q)
                  pinocchio.computeCoriolisMatrix(model, data, q, qdot)
                  pinocchio.rnea(model, data, q, qdot_t, qddot_t)

                  CMM_slice = CMM[:,6:39]
                  COR_slice = data.C[:,6:39]
                  M_slice = data.M[:,6:39]
                  g_slice = data.tau[6:39]

                  modelmsg.CMM = CMM_slice.flatten()
                  modelmsg.COR = COR_slice.flatten()
                  modelmsg.M = M_slice.flatten()
                  modelmsg.g = g_slice.flatten()
            
                  # for i in range(0, 6):
                  #       for j in range(0, 33):
                  #             modelmsg.CMM[33*(i)+j] = CMM[i][j+6]

                  # for i in range(0, 33):
                  #       for j in range(0, 33):
                  #             modelmsg.COR[33*(i)+j] = data.C[i][j+6] 

                  # for i in range(0, 33):
                  #       for j in range(0, 33):
                  #             modelmsg.M[33*(i)+j] = data.M[i][j+6] 

                  # for i in range(0, 33):
                  #       modelmsg.g[i] = data.tau[i+6] 

           #       pub.publish(modelmsg)
           # rate.sleep()

if __name__=='__main__':
      try:
            talker()
      except rospy.ROSInterruptException:
            pass