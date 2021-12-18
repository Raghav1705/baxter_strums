#!/usr/bin/env python
#import dependencies
import rospy
import tf2_ros
# import numpy as np
# import scipy as sp
import sys

def listen(target_frame, source_frame):
	tfBuffer = tf2_ros.Buffer()
	#TF LISTENER, WE ASSUME IS SUBSCRIBING ON ITS OWN
	#IF WE RUN INTO ERROR, CHECK THAT WE NEED TO SUBSCRIBE A DIFF WAY TO TF TOPIC
	tfListener = tf2_ros.TransformListener(tfBuffer) #OUTPUT is type geometry_msgs/TransformStamped documentation on lab doc 1.5
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
			print(trans)
			#WANT TO RETURN OUR POSITION AND ORIENTATION
			#DOESNT NEED TO BE IN SAME FORMAT, BUT DO WANT SAME VALUES

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		  #THIS IS WHERE WE ENTER CODE TO DO IF EXCEPTION OCCURS
		  #IF NO CODE LEAVE IN FNC PASS
		  pass
		# Use our rate object to sleep until it is time to publish again

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  rospy.init_node('tf_echo', anonymous=True)

  try:
    listen(sys.argv[1], sys.argv[2])
  except rospy.ROSInterruptException:
    pass