#!/usr/bin/env python
from __future__ import print_function
import rospy
import roslib; roslib.load_manifest('teleop_twist_keyboard')
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

pub_arms = rospy.Publisher('/tiger/tiger_arm_controller/command',Float64MultiArray, queue_size=10)
pub_wheels = rospy.Publisher('/tiger/tiger_wheel_controller/command',Float64MultiArray, queue_size=10)


def keyboardCallback(key_cmd):
    #do something every time we read in --> map the keys to commands
    l_wheels = key_cmd.linear.x
    r_wheels = key_cmd.linear.y
    arm_data = [key_cmd.linear.z, -key_cmd.angular.x, key_cmd.angular.y, -key_cmd.angular.z,
                -key_cmd.linear.z, key_cmd.angular.x, -key_cmd.angular.y, key_cmd.angular.z,
                -key_cmd.linear.z, key_cmd.angular.x, -key_cmd.angular.y, key_cmd.angular.z]

    wheel_gain=1000
    arm_gain = 1

    #sc:ale and offset values
    l_wheels = wheel_gain*l_wheels
    r_wheels = wheel_gain*r_wheels
    arm_data = arm_data*arm_gain


    wheel_data = [l_wheels, r_wheels, l_wheels, r_wheels]
    wheel_cmd = Float64MultiArray(data=wheel_data)
    arm_cmd = Float64MultiArray(data=arm_data)

    pub_arms.publish(arm_cmd)
    pub_wheels.publish(wheel_cmd)
    #straight -> [100,100,100,100]
    #backwards -> [-100,-100,-100,-100]
    #stop -> [0,0,0,0]
    #turn right -> [100,-100,100,-100]
    #turn left -> [-100,100,-100,100]
    #fwd right -> [150, -50, 150, -50]
    #fwd left -> [-50, 150, -50, 150]

    #raise the body -> [0.8, -0.8, 0.8, -0.8, -0.8, 0.8, -0.8, 0.8, -0.8, 0.8, -0.8, 0.8]
    #lower the body -> [-0.13, 0.13, -0.13, 0.13, 0.13, -0.13, 0.13, -0.13, 0.13, -0.13, 0.13, -0.13]
    # +/-0.1 increments each button press

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('tiger_commander', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, keyboardCallback)
#    pub_arms.publish(arm_cmd)
#    pub_wheels.publish(wheel_cmd)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
