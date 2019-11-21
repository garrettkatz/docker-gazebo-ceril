#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Arm control: Moving body up
---------------------------
   e    r
   d    f

Arm control: SHIFT TO MOVE BODY DOWN
---------------------------
   E    R
   D    F

t : up (+z)
b : down (-z)

anything else : stop

"p/;" : increase/decrease max wheel speeds by 10%

CTRL-C to quit
"""
# 'letter':(left wheels,right wheels,FL arm, FR arm, BL arm, BR arm)
moveBindings = {
        'i':(1,1,0,0,0,0),
        'o':(1,-0.0,0,0,0,0),
        'l':(1,-1,0,0,0,0),
        '.':(-1,0.0,0,0,0,0),
        ',':(-1,-1,0,0,0,0),
        'm':(0.0,-1,0,0,0,0),
        'j':(-1,1,0,0,0,0),
        'u':(-0.0,1,0,0,0,0),
        'k':(0,0,0,0,0,0),


    }

armBindings={
        'e':(0,0,1,0,0,0),
        'r':(0,0,0,1,0,0),
        'd':(0,0,0,0,1,0),
        'f':(0,0,0,0,0,1),
        'E':(0,0,-1,0,0,0),
        'R':(0,0,0,-1,0,0),
        'D':(0,0,0,0,-1,0),
        'F':(0,0,0,0,0,-1),
        's':(0,0,0,0,0,0)
        }
speedBindings={
        'p':(1.1,1.1),
        ';':(.9,.9),
#        'q':(1.1,1.1),
#        'z':(.9,.9),
#        'w':(1.1,1),
#        'x':(.9,1),
#        'e':(1,1.1),
#        'c':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 1.0)  # default speed = 0.5
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    th2 = 0
    th3 = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]

            elif key in armBindings.keys():
                if key == 'e':
                    z = 1
                if key == 'r':
                    th = 1
                if key == 'd':
                    th2 = 1
                if key == 'f':
                    th3 = 1
                if key == 'E':
                    z = -1
                if key == 'R':
                    th = -1
                if key == 'D':
                    th2 = -1
                if key == 'F':
                    th3 = -1
                if key == 's':
                    z=0
                    th=0
                    th2=0
                    th3=0
#                if armBindings[key][5] == 1.0 or armBindings[key][5] == -1.0:
#                    th3 = moveBindings[key][5]
#
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                th2 = 0
                th3 = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z;
            twist.angular.x = th; twist.angular.y = th2; twist.angular.z = th3
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
