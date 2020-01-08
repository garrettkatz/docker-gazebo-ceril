#!/usr/bin/env python
import numpy as np
import rospy
from gazeboEnvironment import GazeboEnvironment
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import tf


class PoppyController(GazeboEnvironment):

    def __init__(self):
        "Initialization function of the turtlebot controller. "
        rospy.loginfo("poppy_application: "+"Start TurtleBotEnv INIT...")

        rospy.loginfo("poppy_application: "+"START init RobotGazeboEnv")
        self._gazebo = GazeboEnvironment(False,"WORLD")
        rospy.loginfo("poppy_application: "+"COMPLETED init RobotGazeboEnv")

        # Joint names is a list that acts as a placeholder for the link names in case we would like to log the output to
        # console
        self._joints_dict = {
            'abs_y': 0,
            'abs_x': 1,
            'abs_z': 2,
            'bust_y': 3,
            'bust_x': 4,
            'head_z': 5,
            'head_y': 6,
            'l_shoulder_y': 7,
            'l_shoulder_x': 8,
            'l_arm_z': 9,
            'l_elbow_y': 10,
            'r_shoulder_y': 11,
            'r_shoulder_x': 12,
            'r_arm_z': 13,
            'r_elbow_y': 14,
            'r_hip_x': 15,
            'r_hip_z': 16,
            'r_hip_y': 17,
            'r_knee_y': 18,
            'r_ankle_y': 19,
            'l_hip_x': 20,
            'l_hip_z': 21,
            'l_hip_y': 22,
            'l_knee_y': 23,
            'l_ankle_y': 24
            }

        # tf_names_dict corresponds to the frames for the links mounted at the joints
        self._tf_names_dict = {
            0: "/abs_motors",
            1: "/abdomen",
            2: "/spine",
            3: "/bust_motors",
            4: "/chest",
            5: "/neck",
            6: "/head",
            7: "/l_shoulder",
            8: "/l_shoulder_motor",
            9: "/l_upper_arm",
            10: "/l_forearm",
            11: "/r_shoulder",
            12: "/r_shoulder_motor",
            13: "/r_upper_arm",
            14: "/r_forearm",
            15: "/r_hip",
            16: "/r_hip_motor",
            17: "/r_thigh",
            18: "/r_shin",
            19: "/r_foot",
            20: "/l_hip",
            21: "/l_hip_motor",
            22: "/l_thigh",
            23: "/l_shin",
            24: "/l_foot"
        }

        self._joint_names = []
        self._joint_angles = np.zeros((25, 1), dtype=np.float64)
        self._tf_listener = tf.TransformListener()

    def _check_imu_ready(self):
        """
        Check if the IMU sensor data is ready. Not being used now
        """
        imu = None
        rospy.loginfo("poppy_application: "+"Waiting for /poppy/joint_states to be READY...")
        while imu is None and not rospy.is_shutdown():
            try:
                imu = rospy.wait_for_message("/imu_data", Imu, timeout=5.0)
                rospy.loginfo("poppy_application: "+"Current /imu_data READY=>")
            except:
                rospy.logerr("Current /imu_data not ready yet, retrying")
        return

    def _check_joint_states(self):
        "Check if the joint states ros topic is ready"
        joint_states = None
        rospy.loginfo("poppy_application: "+"Waiting for /poppy/joint_states to be READY...")
        while joint_states is None and not rospy.is_shutdown():
            try:
                joint_states = rospy.wait_for_message("/poppy/joint_states", JointState , timeout=5.0)
                rospy.loginfo("poppy_application: "+"Current /poppy/joint_states READY=>")
            except:
                rospy.logerr("Current /poppy/joint_states not ready yet, retrying")

        # Populate the joint_names list here once.
        for ind in range(len(joint_states.name)):
            joint_ind = self._joints_dict[joint_states.name[ind]]
            self._joint_names.append(joint_states.name[ind])
        return

    def _check_all_sensors_ready(self):
        "Check if all sensors of the system is initialized"
        rospy.loginfo("poppy_application: "+"START CHECK IF ALL SENSORS READY")
        self._check_joint_states()
        self._check_imu_ready()
        rospy.loginfo("poppy_application: "+"ALL SENSORS READY")

    def _check_tf_listener_ready(self):
        rospy.loginfo("poppy_application: "+"START CHECK IF TF LISTENER READY")
        trans = None
        rot = None
        while rot is None and trans is None:
            rospy.loginfo("poppy_application: "+"START CHECK IF TF LISTENER READY")
            self._tf_listener.waitForTransform("/pelvis", self._tf_names_dict[0], rospy.Time(), rospy.Duration(1.5))
            now = rospy.Time.now()
            try:
                (trans, rot) = self._tf_listener.lookupTransform("/pelvis", self._tf_names_dict[0], now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Current transform lookup not ready yet, retrying")
                continue
        rospy.loginfo("poppy_application: "+"TF LISTENER READY")
        return

    def _joint_states_callback(self, data):
        """
        Callback to update the joint_states
        """
        for ind in range(len(data.name)):
            joint_ind = self._joints_dict[data.name[ind]]
            self._joint_angles[joint_ind] = data.position[ind]
        return

    def _imu_callback(self, data):
        "Callback to update the laser scan ros data"
        self.imu = data

    # def _check_publishers_connection(self):
    #     """
    #     Checks that all the publishers are working
    #     :return:
    #     """
    #     rate = rospy.Rate(10)  # 10hz
    #     while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
    #       rospy.loginfo("poppy_application: ""poppy_application: "+("No susbribers to _cmd_vel_pub yet so we wait and try again")
    #       try:
    #         rate.sleep()
    #       except rospy.ROSInterruptException:
    #         # This is to avoid error when world is rested, time when backwards.
    #         pass
    #     rospy.loginfo("poppy_application: ""poppy_application: "+("_cmd_vel_pub Publisher Connected")
    #     rospy.loginfo("poppy_application: ""poppy_application: "+("All Publishers READY")

    def initialize_controller(self):
        """
        Checks the sensors and actuators and creates subscribers and publishers.
        """
        self._gazebo.unpause_sim()

        # Check all sensors are ready
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/poppy/joint_states", JointState, self._joint_states_callback)
        # IMU Subscriber to be added later

        # self._cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        # self.set_model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=100)
        #
        # self._check_publishers_connection()

        # Adding a TF listener here
        self._check_tf_listener_ready()
        return

    def get_joint_states(self):
        """
        Return the computed joint_angles
        """
        return self._joint_angles, self._joint_names

    def get_position(self):
        """
        Function returns the position of the frames with respect to base
        :return: arrays of orientation and position of size 25
        """
        joint_position = np.zeros((25, 3), dtype=np.float64)
        joint_orientation = np.zeros((25, 4), dtype=np.float64)
        now = rospy.Time.now()
        self._tf_listener.waitForTransform("/pelvis", self._tf_names_dict[0], now, rospy.Duration(1.5))
        for ind in range(25):
            try:
                (trans, rot) = self._tf_listener.lookupTransform("/pelvis", self._tf_names_dict[ind], now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            joint_position[ind, :] = trans
            joint_orientation[ind, :] = rot

        return self._joint_names, joint_position, joint_orientation
