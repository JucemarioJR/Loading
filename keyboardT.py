#!/usr/bin/env python
from __future__ import print_function
import pygame
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from ardrone_autonomy.msg import Navdata
from gazebo_msgs.srv import GetModelState, GetModelPropertiesRequest
from tf.transformations import euler_from_quaternion
from math import atan2
import rospy
import roslib
roslib.load_manifest('teleop_twist_keyboard')


def ReceiverND1(Navdata):
    global status_1
    status_1 = Navdata.state


def ReceiverND2(Navdata):
    global status_2
    status_2 = Navdata.state


def ReceiverND3(Navdata):
    global status_3
    status_3 = Navdata.state


def XYT1(msg):
    global x1
    global y1
    global theta1

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta1) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def XYT2(msg):
    global x2
    global y2
    global theta2

    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta2) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def XYT3(msg):
    global x3
    global y3
    global theta3

    x3 = msg.pose.pose.position.x
    y3 = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta3) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])





def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def ALT1(msg):
    global H1
    H1 = msg.range


def ALT2(msg):
    global H2
    H2 = msg.range


def ALT3(msg):
    global H3
    H3 = msg.range


def Auto_Control(x1, y1, theta1, Partida1, Destino1):
    xa = 0
    ya = 0
    za = 0
    tha = 0
    xi, yi = Partida1
    xf, yf = Destino1

    inc_x = xf - x1
    inc_y = yf - y1
    angle_to_goal = atan2(inc_y, inc_x)

    if angle_to_goal - theta1 > 0.1:
        tha = 0.5
    elif angle_to_goal - theta1 < -0.1:
        tha = -0.5
    else:
        tha = 0
        # relacao do ponto a reta
        position = (xf - xi) * (y1 - yi) - (yf - yi) * \
            (x1 - xi)  # esquerda + e direita -
        d = abs((yf - yi)*x1 + (xi - xf)*y1 + (xf*yi - xi*yf)) / \
            ((yi-yf)**2 + (xf-xi)**2)**(float(1)/2)

        if (d > 0.1 and position > 0):
            ya = -1
        elif d > 0.1 and position < 0:
            ya = 1
        else:
            ya = 0
            if yf - y1 > 0.05:
                xa = 1
            else:
                xa = 0

    return (xa, ya, tha)


def Key_Control(keys, xa, ya, za, tha):
    if keys[pygame.K_w]:
        x = 1
        print("w")
    elif keys[pygame.K_s]:
        x = -1
        print("s")
    else:
        x = xa

    if keys[pygame.K_a]:
        y = 1
        print("a")
    elif keys[pygame.K_d]:
        y = -1
        print("d")
    else:
        y = ya

    if keys[pygame.K_LEFT]:
        th = 1
        print("LEFT")
    elif keys[pygame.K_RIGHT]:
        th = -1
        print("RIGHT")
    else:
        th = tha

    if keys[pygame.K_UP]:
        z = 1
        print("UP")
    elif keys[pygame.K_DOWN]:
        z = -1
        print("Down")
    else:
        z = za

    return (x, y, z, th)


# subscrive for the sonar_height to get the height from the drone
SubAL1 = rospy.Subscriber("/ardrone_1/sonar_height", Range, ALT1)
SubAL2 = rospy.Subscriber("/ardrone_2/sonar_height", Range, ALT2)
SubAL3 = rospy.Subscriber("/ardrone_3/sonar_height", Range, ALT3)

# subscrive for the Navdata to get the status from the drone
SubND1 = rospy.Subscriber("/ardrone_1/navdata", Navdata, ReceiverND1)
SubND2 = rospy.Subscriber("/ardrone_2/navdata", Navdata, ReceiverND2)
SubND3 = rospy.Subscriber("/ardrone_3/navdata", Navdata, ReceiverND3)

# subscrive for the ground_truth to get the position from the drone
SubOD1 = rospy.Subscriber("/ardrone_1/ground_truth/state", Odometry, XYT1)
SubOD2 = rospy.Subscriber("/ardrone_2/ground_truth/state", Odometry, XYT2)
SubOD3 = rospy.Subscriber("/ardrone_3/ground_truth/state", Odometry, XYT3)

# Publisher in the commands from the drone for control theys
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

# Create a node with this name in the ros server
rospy.init_node('joao')

speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 1.0)

xa, xb, xc = 0 ,0 ,0
ya, yb, yc = 0 ,0 ,0
za, zb, zc = 0 ,0 ,0
tha, thb, thc = 0 ,0 ,0
status = 0

Partida1 = (6, -8)
Destino1 = (6, 8)

Partida2 = (2, -8)
Destino2 = (2, 8)

Partida3 = (-2, -8)
Destino3 = (-2, 8)

r = rospy.Rate(4)

pygame.init()

pygame.display.set_mode((800, 300))

looping = True

ConDrone = 0

while looping:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            looping = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_v:
                if ConDrone == 1:
                    pubo1.publish(vazio)
                    print(status_1)
                elif ConDrone == 2:
                    pubo2.publish(vazio)
                elif ConDrone == 3:
                    pubo3.publish(vazio)    
            if event.key == pygame.K_b:
                if ConDrone == 1:
                    publ1.publish(vazio)
                    print(status_1)
                elif ConDrone == 2:
                    publ2.publish(vazio)
                elif ConDrone == 3:
                    publ3.publish(vazio)
            if event.key == pygame.K_1:
                print("Botao 1")
                ConDrone = 1
            if event.key == pygame.K_2:
                print("Botao 2")
                ConDrone = 2
            if event.key == pygame.K_3:
                print("Botao 3")
                ConDrone = 3
            if event.key == pygame.K_4:
                print("Botao 4")
                ConDrone = 4
    if ConDrone == 4:
        pubo1.publish(vazio)
        pubo2.publish(vazio)
        pubo3.publish(vazio)
    if ConDrone == 0:
        publ1.publish(vazio)
        publ2.publish(vazio)
        publ3.publish(vazio)

    xa, ya, tha = Auto_Control(x1, y1, theta1, Partida1, Destino1)
    xb, yb, thb = Auto_Control(x2, y2, theta2, Partida2, Destino2)
    xc, yc, thc = Auto_Control(x3, y3, theta3, Partida3, Destino3)

    keys = pygame.key.get_pressed()

    if ConDrone == 1:
        xa, ya, za, tha = Key_Control(keys, xa, ya, za, tha)
    elif ConDrone == 2:
        xb, yb, zb, thb = Key_Control(keys, xb, yb, zb, thb)
    elif ConDrone == 3:
        xc, yc, zc, thc = Key_Control(keys, xc, yc, zc, thc)

    twist1 = Twist()
    twist2 = Twist()
    twist3 = Twist()
    twist1.linear.x = xa*speed
    twist1.linear.y = ya*speed
    twist1.linear.z = za*speed
    twist1.angular.x = 0
    twist1.angular.y = 0
    twist1.angular.z = tha*turn
    pub1.publish(twist1)

    twist2.linear.x = xb*speed
    twist2.linear.y = yb*speed
    twist2.linear.z = zb*speed
    twist2.angular.x = 0
    twist2.angular.y = 0
    twist2.angular.z = thb*turn
    pub2.publish(twist2)

    twist3.linear.x = xc*speed
    twist3.linear.y = yc*speed
    twist3.linear.z = zc*speed
    twist3.angular.x = 0
    twist3.angular.y = 0
    twist3.angular.z = thc*turn
    pub3.publish(twist3)

    r.sleep()
pygame.quit()
