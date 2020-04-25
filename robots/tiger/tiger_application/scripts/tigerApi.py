import numpy as np
import rospy
from tigerController import *


class TigerApi:
    def __init__(self):
        self._tiger_controller = None
        return

    def connect(self):
        rospy.loginfo("tiger_application:"+"Initiating connection to tiger")
        self._tiger_controller = TigerController()
        self._tiger_controller.initialize_controller()
        rospy.loginfo("tiger_application:"+"Connection to tiger completed")
        return

    def get_states(self):
        """
        Return the computed joint_angles
        """
        return self._tiger_controller.get_joint_states()

    def get_mobile_base_state(self):
        return self._tiger_controller.get_mobile_base_states()

    def get_position(self):
        """
        Function returns the position of the frames with respect to base
        :return: arrays of orientation and position of size 16
        """
        return self._tiger_controller.get_position()

    def set_joint_states(self):
        """
        Function sets the UR10 arm joints to the required angles
        :return:
        """
        ur10_joints = []
        ur10_shoulder_pan = input("Enter the shoulder pan joint angle in rads: -1.7 to 1.7\n")
        if ur10_shoulder_pan > 1.7 or ur10_shoulder_pan < -1.7:
            ur10_shoulder_pan = 0
        ur10_joints.append(ur10_shoulder_pan)

        ur10_shoulder_lift = input("Enter the shoulder lift joint angle in rads: -1.7 to 0\n")
        if ur10_shoulder_lift > 0 or ur10_shoulder_lift < -1.7:
            ur10_shoulder_lift = 0
        ur10_joints.append(ur10_shoulder_lift)

        ur10_elbow_joint = input("Enter the elbow joint angle in rads: -1.7 to 0\n")
        if ur10_elbow_joint > 0 or ur10_elbow_joint < -1.7:
            ur10_elbow_joint = 0
        ur10_joints.append(ur10_elbow_joint)

        ur10_wrist_1_joint = input("Enter the wrist_1_joint angle in rads: -1.7 to 1.7\n")
        if ur10_wrist_1_joint > 1.7 or ur10_wrist_1_joint < -1.7:
            ur10_wrist_1_joint = 0
        ur10_joints.append(ur10_wrist_1_joint)

        ur10_wrist_2_joint = input("Enter the wrist_2_joint angle in rads: -1.7 to 1.7\n")
        if ur10_wrist_2_joint > 1.7 or ur10_wrist_2_joint < -1.7:
            ur10_wrist_2_joint = 0
        ur10_joints.append(ur10_wrist_2_joint)

        ur10_wrist_3_joint = input("Enter the wrist_2_joint angle in rads: -1.7 to 1.7\n")
        if ur10_wrist_3_joint > 1.7 or ur10_wrist_3_joint < -1.7:
            ur10_wrist_3_joint = 0
        ur10_joints.append(ur10_wrist_3_joint)

        print("Displayed joints are:")
        print(ur10_joints)
        self._tiger_controller.set_joint_states(ur10_joints)

    def set_mobile_base(self):
        base_position = []
        base_position_x = input("Enter the x coordinate of the mobile base")
        base_position.append(base_position_x)

        base_position_y = input("Enter the y coordinate of the mobile base")
        base_position.append(base_position_y)

        self._tiger_controller.set_mobile_state(base_position, [])

    def close(self):
        """
        Function closes the connection to gazebo
        :return: arrays of orientation and position of size 16
        """
        rospy.loginfo("Closing the tiger robot")
        self._tiger_controller = None
