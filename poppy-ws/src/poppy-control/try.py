#!/usr/bin/env python
'''
import rospy
from geometry_msgs.msg import Pose

'''
'''
name: [ground_plane, robot]
pose:
  -
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  -
    position:
      x: -0.307810391073
      y: 0.238960349769
      z: 0.036496355644
    orientation:
      x: 0.0607023684758
      y: -0.693777348281
      z: 0.717313667038
      w: -0.0211970883482
twist:
  -
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  -
    linear:
      x: 0.000533375561194
      y: -0.00022637674781
      z: 0.00117413674143
    angular:
      x: -0.00124014824684
      y: -0.0482095305591
      z: 0.00155201300952
'''
'''

def main():
    pub = rospy.Publisher('gazebo/set_model_state', Pose, queue_size=10)
    rospy.init_node('commander', anonymous=True)

    rate = rospy.Rate(2)
    msg = Pose()
    msg.position.x = -0.307810391073
    msg.position.y = -3.238960349769
    msg.position.z = 0.036496355644
    msg.orientation.x = 0.0607023684758
    msg.orientation.y = -0.693777348281
    msg.orientation.z = 0.717313667038
    msg.orientation.w = -0.0211970883482
    count = 0
    while not rospy.is_shutdown():
        if count < 15:
            msg.position.y += .5
            pub.publish(msg)
            rate.sleep()
            count = count + 1
        else:
            count = 0
            msg.position.y = -3.238960349769
            pub.publish(msg)
            rate.sleep()

if __name__== '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
'''


import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')

    rate = rospy.Rate(2)
    state_msg = ModelState()
    state_msg.model_name = 'Poppy_Humanoid'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.3
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        count = 0
        while not rospy.is_shutdown():
            if count < 15:
                state_msg.pose.position.y += .5
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )
                #pub.publish(msg)
                rate.sleep()
                count = count + 1
            else:
                count = 0
                state_msg.pose.position.y = -3.238960349769
                #pub.publish(msg)
                rate.sleep()


    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
