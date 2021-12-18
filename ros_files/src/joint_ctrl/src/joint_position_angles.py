#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import numpy as np
import argparse
import rospy
import time

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

def init():
    epilog = """
        See help inside the example with the '?' key for key bindings.
    """
    # rp = baxter_interface.RobotParams()
    # valid_limbs = rp.get_limb_names()
    # if not valid_limbs:
    #     rp.log_message(("Cannot detect any limb parameters on this robot. "
    #                     "Exiting."), "ERROR")
    #     return
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__,
    #                                  epilog=epilog)
    # parser.add_argument(
    #     "-l", "--limb", dest="limb", help="Limb on which to run the joint position keyboard example"
    # )

    # args = parser.parse_args(rospy.myargv()[1:])
    # print("Initializing node... ")
    # rospy.init_node("strum_ukelele")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    rospy.loginfo("Enabling robot...")
    rs.enable()

    rospy.on_shutdown(clean_shutdown)

def clean_shutdown():
        print("\nExiting example.")

    
def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    limb = 'left'
    joint_state_dict = {
        "{}_s0".format(limb): -.517,#-1.0,
        "{}_s1".format(limb): -.092,#-0.39,
        "{}_e0".format(limb): -1.577,#0.15,
        "{}_e1".format(limb): 1.35,#1,
        "{}_w0".format(limb): -.022,#-0.2,
        "{}_w1".format(limb): 1.9,#0.9,
        "{}_w2".format(limb): 2.64,#0.7,
    }
    for joint, angle in joint_state_dict.items():
        set_j(limb, joint, angle, joint_state_dict)
    time.sleep(1)
    joint_state_dict = {
        "{}_s0".format(limb): -.517,#-1.0,
        "{}_s1".format(limb): -.022,#-0.34,
        "{}_e0".format(limb): -1.475,#0.15,
        "{}_e1".format(limb): 1.45,#1,
        "{}_w0".format(limb): -.022,#-0.2,
        "{}_w1".format(limb): 1.9,#0.9,
        "{}_w2".format(limb): 2.64,#0.7,
    }
    for joint, angle in joint_state_dict.items():
        set_j(limb, joint, angle, joint_state_dict)
    time.sleep(1)
    joint_state_dict = {
        "{}_s0".format(limb): -.517,#-1.0,
        "{}_s1".format(limb): -.022,#-0.268,
        "{}_e0".format(limb): -1.467,#0.15,
        "{}_e1".format(limb): 1.5,#1,
        "{}_w0".format(limb): -.022,#-0.2,
        "{}_w1".format(limb): 1.9,#0.9,
        "{}_w2".format(limb): 2.64,#0.75,
    }
    for joint, angle in joint_state_dict.items():
        set_j(limb, joint, angle, joint_state_dict)
    return 'Done'

def strum_up():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    limb = 'left'

    joint_state_dict = {
        "{}_s0".format(limb): -.517, #-1.0,
        "{}_s1".format(limb): -.022, #-0.268,
        "{}_e0".format(limb): -1.467, #0.151,
        "{}_e1".format(limb): 1.5, #1,
        "{}_w0".format(limb): -.022, #-0.2,
        "{}_w1".format(limb): 1.9, #0.9,
        "{}_w2".format(limb): 0.8, #-0.45,
    }
    for joint, angle in joint_state_dict.items():
        set_j(limb, joint, angle, joint_state_dict)

    return 'Done'    

def strum_down():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    limb = 'left'
    joint_state_dict = {
        "{}_s0".format(limb): -.517,
        "{}_s1".format(limb): -.022,
        "{}_e0".format(limb): -1.467,
        "{}_e1".format(limb): 1.5, 
        "{}_w0".format(limb): -.022,
        "{}_w1".format(limb): 1.9, 
        "{}_w2".format(limb): 2.64, 
    }
    for joint, angle in joint_state_dict.items():
         set_j(limb, joint, angle, joint_state_dict)
    # joint_state_dict = {
    #     "{}_s0".format(limb): -1.0,
    #     "{}_s1".format(limb): -0.28,
    #     "{}_e0".format(limb): 0.15,
    #     "{}_e1".format(limb): 1,
    #     "{}_w0".format(limb): -0.2,
    #     "{}_w1".format(limb): 0.9,
    #     "{}_w2".format(limb): 0.75,
    # }
    # for joint, angle in joint_state_dict.items():
    #     set_j(limb, joint, angle, joint_state_dict)
    # for joint, angle in joint_state_dict.items():
    #     set_j(limb, joint, angle, joint_state_dict)
    # joint_state_dict = {
    #     "{}_s0".format(limb): -1.0,
    #     "{}_s1".format(limb): -0.268,
    #     "{}_e0".format(limb): 0.15,
    #     "{}_e1".format(limb): 1,
    #     "{}_w0".format(limb): -0.2,
    #     "{}_w1".format(limb): 0.9,
    #     "{}_w2".format(limb): 0.75,
    # }
    # for joint, angle in joint_state_dict.items():
    #     set_j(limb, joint, angle, joint_state_dict)
    time.sleep(1)
    return 'Done'   

def set_j(side, joint_name, new_position, joint_state_dict):
    rate = rospy.Rate(100)
    limb = baxter_interface.Limb(side)
    current_position = limb.joint_angle(joint_name)
    while not np.isclose(current_position, new_position, atol=0.01):
        joint_command = {joint_name: new_position}
        limb.set_joint_positions(joint_state_dict)
        current_position = limb.joint_angle(joint_name)
        rate.sleep()

# if __name__ == '__main__':
#     init()
#     main()
#     while True:
#         x = raw_input("Press Enter to continue...")
#         strum_up()
#         x = raw_input("Press Enter to continue...")
#         strum_down()