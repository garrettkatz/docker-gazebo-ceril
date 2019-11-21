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

    def get_position(self):
        """
        Function returns the position of the frames with respect to base
        :return: arrays of orientation and position of size 16
        """
        return self._tiger_controller.get_position()

    def close(self):
        """
        Function closes the connection to gazebo
        :return: arrays of orientation and position of size 16
        """
        self._tiger_controller = None
