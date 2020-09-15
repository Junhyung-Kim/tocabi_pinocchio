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
      rospy.Subscriber("/tocabi/jointstates", JointState, callback)
      global model
      pub = rospy.Publisher("/tocabi/pinocchio", model, queue_size=1)
      modelmsg.CMM = [0 for i in range(33*6)]
      modelmsg.COR = [0 for i in range(33*6)] 
      rate = rospy.Rate(50) #10hz
      model = pinocchio.buildModelFromUrdf("/home/jhk/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi_sim.urdf")      
      data = model.createData()
      global start
      q = pinocchio.randomConfiguration(model)
      qdot = pinocchio.randomConfiguration(model)
      start = True
      while not rospy.is_shutdown():
            if start:
                  global qmsg
                  for i in range(0, 33):
                        #q[i] = qmsg.position[i]
                        q[i] = 1.0
                        qdot[i] = 1.0
                  #      qdot[i] = qmsg.velocity[i]
                   
                  CMM = pinocchio.computeCentroidalMap(model, data, q)
                  pinocchio.computeCoriolisMatrix(model,data,q,qdot)
                  pinocchio.computeAllTerms(model,data,q,qdot)
                  g = pinocchio.computeGeneralizedGravity(model,data,q)
                  #data.nle

                  for i in range(0, 6):
                        for j in range(0, 33):
                              modelmsg.CMM[33*(i)+j] = CMM[i][j] 

                  for i in range(0, 6):
                        for j in range(0, 33):
                              modelmsg.COR[33*(i)+j] = data.C[i][j] 

                  print("Cori")
                  print(data.C.shape)
                  #print(np.multiply(data.C,qdot))
                  print("aa")
                  print(data.nle - g)

                  pub.publish(modelmsg)
                  rate.sleep()

if __name__=='__main__':
      try:
            talker()
      except rospy.ROSInterruptException:
            pass