#! /usr/bin/env python

import rospy
import numpy as np
from tigerApi import *
from enum import IntEnum

class UserInput(IntEnum):
    GET_JOINT_STATES = 1
    GET_JOINT_POSITION = 2
    SET_JOINT_STATES = 3
    SET_JOINT_POSITION = 4
    EXIT = 5

# Initiate the rospy node
rospy.init_node("tiger_application_1", anonymous=False, log_level=rospy.INFO, disable_signals=False)

print("tiger_application: "+"Beginning of file")

rospy.loginfo("tiger_application: "+"Instantiating Tiger Controller")
tigerRobot = TigerApi()
rospy.loginfo("tiger_application: "+"Completed Tiger Controller Instantiation")
tigerRobot.connect()
rospy.loginfo("tiger_application: "+"Connected tiger controller. Getting states")

rospy.loginfo("tiger_application: Getting user input for API")

usrInput =  UserInput.GET_JOINT_STATES

while usrInput != UserInput.EXIT:

    rospy.loginfo("")
    rospy.loginfo("")
    rospy.loginfo("Enter the following input")
    rospy.loginfo(str(int(UserInput.GET_JOINT_STATES)) + ". Get Joint States")
    rospy.loginfo(str(int(UserInput.GET_JOINT_POSITION)) + ". Get Joint Position")
    rospy.loginfo(str(int(UserInput.SET_JOINT_STATES)) + ". Set Joint States")
    rospy.loginfo(str(int(UserInput.EXIT)) + ". Exit Program")
    usrInput = None
    while not usrInput:
        try:
            usrInput = int(raw_input())
        except ValueError:
            rospy.loginfo("Invalid Number")

    if usrInput == UserInput.GET_JOINT_STATES:
        rospy.loginfo("tiger_application: "+"Querying joint states")
        joint_states, joint_names = tigerRobot.get_states()
        for ind in range(np.size(joint_states, 0)):
            rospy.loginfo("%s: %s [rad]", joint_names[ind], joint_states[ind])

    elif usrInput == UserInput.GET_JOINT_POSITION:
        rospy.loginfo("tiger_application: "+"Querying joint positions")
        joint_names, joint_position, joint_orientation = tigerRobot.get_position()

        for ind in range(np.size(joint_states, 0)):
            rospy.loginfo("%s, position: %s [m], orientation: %s", joint_names[ind], joint_position[ind], joint_orientation[ind])

    elif usrInput == UserInput.SET_JOINT_STATES:
        tigerRobot.set_joint_states()

    elif usrInput == UserInput.EXIT:
        tigerRobot.close()

rospy.signal_shutdown("Closing the tiger_application node")
