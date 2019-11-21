#! /usr/bin/env python

import rospy
import numpy as np
from tigerApi import *

# Initiate the rospy node
rospy.init_node("tiger_application_1", anonymous=False, log_level=rospy.INFO, disable_signals=False)

print("tiger_application: "+"Beginning of file")

rospy.loginfo("tiger_application: "+"Instantiating Tiger Controller")
tigerRobot = TigerApi()
rospy.loginfo("tiger_application: "+"Completed Tiger Controller Instantiation")
tigerRobot.connect()
rospy.loginfo("tiger_application: "+"Connected tiger controller. Getting states")

for ind in range(5):
    rospy.loginfo("tiger_application: "+"Querying joint states")
    joint_states, joint_names = tigerRobot.get_states()
    for ind in range(np.size(joint_states, 0)):
        rospy.loginfo("%s: %s [rad]", joint_names[ind], joint_states[ind])

    rospy.loginfo("tiger_application: "+"Querying joint positions")
    joint_names, joint_position, joint_orientation = tigerRobot.get_position()

    for ind in range(np.size(joint_states, 0)):
        rospy.loginfo("%s, position: %s [m], orientation: %s", joint_names[ind], joint_position[ind], joint_orientation[ind])

tigerRobot.close()
rospy.signal_shutdown("Closing the tiger_application node")
