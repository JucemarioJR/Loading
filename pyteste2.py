#!/usr/bin/env python

from __future__ import print_function
import rospy
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import tty
import termios
import select
import sys
from math import atan2
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState, GetModelPropertiesRequest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def XYT1(msg):
    global x1
    global y1
    global theta1

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def XYT2(msg):
    global x2
    global y2
    global theta2

    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta2) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def XYT3(msg):
    global x3
    global y3
    global theta3

    x3 = msg.pose.pose.position.x
    y3 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta3) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def ALT1(msg):
    global H1
    H1 = msg.range
   
def ALT2(msg):
    global H2
    H2 = msg.range

def ALT3(msg):
    global H3
    H3 = msg.range


moveBindings = {
    'i': (1, 0, 0, 0),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'k': (-1, 0, 0, 0),
    'u': (1, 1, 0, 0),
    'm': (-1, 1, 0, 0),
    'o': (1, -1, 0, 0),
    ',': (-1, -1, 0, 0),
    'w': (0, 0, 1, 0),
    's': (0, 0, -1, 0),
    'd': (0, 0, 0, -1),
    'a': (0, 0, 0, 1),
}

speedBindings = {
    'z': (1.1, 1.1),
    'x': (.9, .9),
    'c': (1.1, 1),
    'v': (.9, 1),
    'b': (1, 1.1),
    'n': (1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    
    settings = termios.tcgetattr(sys.stdin)
    #sub = rospy.Subscriber("/odometry/filtered", Odometry)
    #PubOD = rospy.Publisher("/odometry", Odometry, queue_size=1)
    #rospy.wait_for_service('/gazebo/get_model_state')
    #get_model_srv = rospy.ServiceProxy(
    #    '/gazebo/get_model_state', GetModelState)
    
    #model = GetModelState._request_class()
    #model.model_name = 'quadrotor'

       
    SubAL1 = rospy.Subscriber("/ardrone_1/sonar_height", Range, ALT1)
    SubAL2 = rospy.Subscriber("/ardrone_2/sonar_height", Range, ALT2)
    SubAL3 = rospy.Subscriber("/ardrone_3/sonar_height", Range, ALT3)


    SubOD1 = rospy.Subscriber("/ardrone_1/ground_truth/state", Odometry, XYT1)
    SubOD2 = rospy.Subscriber("/ardrone_2/ground_truth/state", Odometry, XYT2)
    SubOD3 = rospy.Subscriber("/ardrone_3/ground_truth/state", Odometry, XYT3)
    
    pub1 = rospy.Publisher('/ardrone_1/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/ardrone_2/cmd_vel', Twist, queue_size=1)
    pub3 = rospy.Publisher('/ardrone_3/cmd_vel', Twist, queue_size=1)
    pubo1 = rospy.Publisher('/ardrone_1/takeoff', Empty, queue_size=1)
    pubo2 = rospy.Publisher('/ardrone_2/takeoff', Empty, queue_size=1)
    pubo3 = rospy.Publisher('/ardrone_3/takeoff', Empty, queue_size=1)
    publ1 = rospy.Publisher('/ardrone_1/land', Empty, queue_size=1)
    publ2 = rospy.Publisher('/ardrone_2/land', Empty, queue_size=1)
    publ3 = rospy.Publisher('/ardrone_3/land', Empty, queue_size=1)
    vazio = Empty()
    
    rospy.init_node('pyteste2')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    r = rospy.Rate(4)
    try:
        # print(msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
          
            if key in moveBindings.keys():
                print(x1)
                print(y1)
                print(x2)
                print(y2)
                print(x3)
                print(y3)
                print(H1)
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == '1':
                pubo1.publish(vazio)
            elif key == '2':
                publ1.publish(vazio)
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed
            twist.linear.y = y*speed
            twist.linear.z = z*speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th*turn
            pub1.publish(twist)
            r.sleep()
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub1.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
