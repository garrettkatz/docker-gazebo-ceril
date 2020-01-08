import numpy as np
import rospy
from poppyController import *


class PoppyApi:
    def __init__(self):
        self._poppy_controller = None
        return

    def connect(self):
        rospy.loginfo("poppy_application:"+"Initiating connection to poppy")
        self._poppy_controller = PoppyController()
        self._poppy_controller.initialize_controller()
        rospy.loginfo("poppy_application:"+"Connection to poppy completed")
        return

    def get_states(self):
        """
        Return the computed joint_angles
        """
        return self._poppy_controller.get_joint_states()

    def get_position(self):
        """
        Function returns the position of the frames with respect to base
        :return: arrays of orientation and position of size 16
        """
        return self._poppy_controller.get_position()

    def close(self):
        """
        Function closes the connection to gazebo
        :return: arrays of orientation and position of size 16
        """
        self._poppy_controller = None
