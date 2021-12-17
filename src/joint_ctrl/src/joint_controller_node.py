#!/usr/bin/env python
#import dependencies
import rospy
# import numpy as np
# import scipy as sp
# import kin_func_skeleton as kfs
import time
import joint_position_angles as jpa
from std_msgs.msg import UInt16

def clean_shutdown():
        print("\nExiting example.")

def callback(msg):
	if msg.data == 1:
		jpa.strum_up()
		time.sleep(0.2)
		jpa.strum_down()
		print('Strum Done')

	rospy.on_shutdown(clean_shutdown)

def joint_listener():
	jpa.init()
	jpa.main()
	#jpa.strum_up()
	#time.sleep(0.2)
	#jpa.strum_down()
	rospy.Subscriber("strum_time", UInt16, callback)
	rospy.spin()


if __name__ == '__main__':
	rospy.init_node('joint_listener', anonymous = True)
	joint_listener()
