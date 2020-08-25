#!/usr/bin/env python 
import rospy
from std_msgs.msg import String

import pinocchio
from sys import argv
from os.path import dirname, join, abspath

def talker():
      pub = rospy.Publisher('chatter',String,queue_size=10)
      rospy.init_node('talker', anonymous = True)
      rate = rospy.Rate(10) #10hz
      model = pinocchio.buildModelFromUrdf("/home/jhk/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi_sim.urdf")
      data     = model.createData()
 
      while not rospy.is_shutdown():
            q = pinocchio.randomConfiguration(model)
            qdot = pinocchio.randomConfiguration(model)
            pinocchio.forwardKinematics(model,data,q)

	    CMM = pinocchio.computeCentroidalMap(model, data, q)
            print(1)
            print(CMM)
            rate.sleep()

if __name__=='__main__':
      try:
            talker()
      except rospy.ROSInterruptException:
            pass

# This path refers to Pinocchio source code but you can define your own directory here.
#pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

# You should change here to set up your own URDF file or just pass it as an argument of this example.
#urdf_filename = pinocchio_model_dir + '/others/robots/ur_description/urdf/ur5_robot.urdf' if len(argv)<2 else argv[1]

# Load the urdf model
#model    = pinocchio.buildModelFromUrdf(urdf_filename)
#print('model name: ' + model.name)

# Create data required by the algorithms
#data     = model.createData()

# Sample a random configuration
#q        = pinocchio.randomConfiguration(model)
#print('q: %s' % q.T)

# Perform the forward kinematics over the kinematic tree
#pinocchio.forwardKinematics(model,data,q)

# Print out the placement of each joint of the kinematic tree
#for name, oMi in zip(model.names, data.oMi):
#    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
#          .format( name, *oMi.translation.T.flat )))
