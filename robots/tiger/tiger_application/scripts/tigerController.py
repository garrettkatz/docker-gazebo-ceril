#!/usr/bin/env python
import numpy as np
import rospy
from gazeboEnvironment import GazeboEnvironment
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import tf


class TigerController(GazeboEnvironment):

    def __init__(self):
        "Initialization function of the turtlebot controller. "
        rospy.loginfo("tiger_application: "+"Start TurtleBotEnv INIT...")

        rospy.loginfo("tiger_application: "+"START init RobotGazeboEnv")
        self._gazebo = GazeboEnvironment(False,"WORLD")
        rospy.loginfo("tiger_application: "+"COMPLETED init RobotGazeboEnv")

        # Joint names is a list that acts as a placeholder for the link names in case we would like to log the output to
        # console
        self._joints_dict = {
            'base_link_lower_arm_link_joint1': 0,
            'base_link_lower_arm_link_joint2': 1,
            'base_link_lower_arm_link_joint3': 2,
            'base_link_lower_arm_link_joint4': 3,
            'base_link_upper_arm_link_joint1': 4,
            'base_link_upper_arm_link_joint2': 5,
            'base_link_upper_arm_link_joint3': 6,
            'base_link_upper_arm_link_joint4': 7,
            'lower_arm_link_wheel_mount_link_joint1': 8,
            'lower_arm_link_wheel_mount_link_joint2': 9,
            'lower_arm_link_wheel_mount_link_joint3': 10,
            'lower_arm_link_wheel_mount_link_joint4': 11,
            'ur10_custom_elbow_joint': 12,
            'ur10_custom_shoulder_lift_joint': 13,
            'ur10_custom_shoulder_pan_joint': 14,
            'ur10_custom_wrist_1_joint': 15,
            'ur10_custom_wrist_2_joint': 16,
            'ur10_custom_wrist_3_joint': 17,
            'wheel_mount_link_FR_wheel_link_joint1': 18,
            'wheel_mount_link_FR_wheel_link_joint2': 19,
            'wheel_mount_link_FR_wheel_link_joint3': 20,
            'wheel_mount_link_FR_wheel_link_joint4': 21
        }

        # tf_names_dict corresponds to the frames for the links mounted at the joints
        self._tf_names_dict = {
            0: "/lower_arm_link1",
            1: "/lower_arm_link2",
            2: "/lower_arm_link3",
            3: "/lower_arm_link4",
            4: "/upper_arm_link1",
            5: "/upper_arm_link2",
            6: "/upper_arm_link3",
            7: "/upper_arm_link4",
            8: "/wheel_mount_link1",
            9: "/wheel_mount_link2",
            10: "/wheel_mount_link3",
            11: "/wheel_mount_link4",
            12: "/FR_wheel_link1",
            13: "/FR_wheel_link2",
            14: "/FR_wheel_link3",
            15: "/FR_wheel_link4",
            16: "/ur10_custom_base_link",
            17: "/ur10_custom_shoulder_link",
            18: "/ur10_custom_upper_arm_link",
            19: "/ur10_custom_forearm_link",
            20: "/ur10_custom_wrist_1_link",
            21: "/ur10_custom_wrist_2_link",
            22: "/ur10_custom_wrist_3_link"
        }

        self._joint_names = []
        self._joint_angles = np.zeros((16, 1), dtype=np.float64)
        self._tf_listener = tf.TransformListener()

    def _check_imu_ready(self):
        """
        Check if the IMU sensor data is ready. Not being used now
        """
        imu = None
        rospy.loginfo("tiger_application: "+"Waiting for /tiger/joint_states to be READY...")
        while imu is None and not rospy.is_shutdown():
            try:
                imu = rospy.wait_for_message("/imu_data", Imu, timeout=5.0)
                rospy.loginfo("tiger_application: "+"Current /imu_data READY=>")
            except:
                rospy.logerr("Current /imu_data not ready yet, retrying")
        return

    def _check_joint_states(self):
        "Check if the joint states ros topic is ready"
        joint_states = None
        rospy.loginfo("tiger_application: "+"Waiting for /tiger/joint_states to be READY...")
        while joint_states is None and not rospy.is_shutdown():
            try:
                joint_states = rospy.wait_for_message("/tiger/joint_states", JointState , timeout=5.0)
                rospy.loginfo("tiger_application: "+"Current /tiger/joint_states READY=>")
            except:
                rospy.logerr("Current /tiger/joint_states not ready yet, retrying")

        # Populate the joint_names list here once.
        for ind in range(len(joint_states.name)):
            joint_ind = self._joints_dict[joint_states.name[ind]]
            self._joint_names.append(joint_states.name[ind])
        return

    def _check_all_sensors_ready(self):
        "Check if all sensors of the system is initialized"
        rospy.loginfo("tiger_application: "+"START CHECK IF ALL SENSORS READY")
        self._check_joint_states()
        self._check_imu_ready()
        rospy.loginfo("tiger_application: "+"ALL SENSORS READY")

    def _check_tf_listener_ready(self):
        rospy.loginfo("tiger_application: "+"START CHECK IF TF LISTENER READY")
        trans = None
        rot = None
        while rot is None and trans is None:
            rospy.loginfo("tiger_application: "+"START CHECK IF TF LISTENER READY")
            self._tf_listener.waitForTransform("/base_link", self._tf_names_dict[0], rospy.Time(), rospy.Duration(1.5))
            now = rospy.Time.now()
            try:
                (trans, rot) = self._tf_listener.lookupTransform("/base_link", self._tf_names_dict[0], now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Current transform lookup not ready yet, retrying")
                continue
        rospy.loginfo("tiger_application: "+"TF LISTENER READY")
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
    #       rospy.loginfo("tiger_application: ""tiger_application: "+("No susbribers to _cmd_vel_pub yet so we wait and try again")
    #       try:
    #         rate.sleep()
    #       except rospy.ROSInterruptException:
    #         # This is to avoid error when world is rested, time when backwards.
    #         pass
    #     rospy.loginfo("tiger_application: ""tiger_application: "+("_cmd_vel_pub Publisher Connected")
    #     rospy.loginfo("tiger_application: ""tiger_application: "+("All Publishers READY")

    def initialize_controller(self):
        """
        Checks the sensors and actuators and creates subscribers and publishers.
        """
        self._gazebo.unpause_sim()

        # Check all sensors are ready
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/tiger/joint_states", JointState, self._joint_states_callback)
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
        :return: arrays of orientation and position of size 16
        """
        joint_position = np.zeros((16, 3), dtype=np.float64)
        joint_orientation = np.zeros((16, 4), dtype=np.float64)
        now = rospy.Time.now()
        self._tf_listener.waitForTransform("/base_link", self._tf_names_dict[0], now, rospy.Duration(1.5))
        for ind in range(16):
            try:
                (trans, rot) = self._tf_listener.lookupTransform("/base_link", self._tf_names_dict[ind], now)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            joint_position[ind, :] = trans
            joint_orientation[ind, :] = rot

        return self._joint_names, joint_position, joint_orientation
