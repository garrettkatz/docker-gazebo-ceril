#! /usr/bin/env python

import rospy
import numpy as np
from poppyApi import *

# Initiate the rospy node
rospy.init_node("poppy_application_1", anonymous=False, log_level=rospy.INFO, disable_signals=False)

print("poppy_application: "+"Beginning of file")

rospy.loginfo("poppy_application: "+"Instantiating Poppy Controller")
poppyRobot = PoppyApi()
rospy.loginfo("poppy_application: "+"Completed Poppy Controller Instantiation")
poppyRobot.connect()
rospy.loginfo("poppy_application: "+"Connected poppy controller. Getting states")

for ind in range(5):
    rospy.loginfo("poppy_application: "+"Querying joint states")
    joint_states, joint_names = poppyRobot.get_states()
    for ind in range(np.size(joint_states, 0)):
        rospy.loginfo("%s: %s [rad]", joint_names[ind], joint_states[ind])

    rospy.loginfo("poppy_application: "+"Querying joint positions")
    joint_names, joint_position, joint_orientation = poppyRobot.get_position()

    for ind in range(np.size(joint_states, 0)):
        rospy.loginfo("%s, position: %s [m], orientation: %s", joint_names[ind], joint_position[ind], joint_orientation[ind])

poppyRobot.close()
rospy.signal_shutdown("Closing the poppy_application node")
