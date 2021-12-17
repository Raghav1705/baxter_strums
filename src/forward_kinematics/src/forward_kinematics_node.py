#!/usr/bin/env python
#import dependencies
import rospy
# import numpy as np
# import scipy as sp
# import kin_func_skeleton as kfs
import baxter_forward_kinematics as bfk
from sensor_msgs.msg import JointState

def callback(joint_state):
	x = bfk.baxter_forward_kinematics_from_joint_state(joint_state)
	print(x)
def joint_listener():
	rospy.Subscriber("robot/joint_states", JointState, callback)
	rospy.spin()



if __name__ == '__main__':
	rospy.init_node('joint_listener', anonymous = True)
	joint_listener()
