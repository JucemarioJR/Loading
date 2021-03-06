#!/usr/bin/env python
from __future__ import print_function
import pygame
from std_srvs.srv import Empty
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

#Modulos para obtencao dos status do drone
def ReceiverND1(Navdata):
    global status_1
    status_1 = Navdata.state


def ReceiverND2(Navdata):
    global status_2
    status_2 = Navdata.state


def ReceiverND3(Navdata):
    global status_3
    status_3 = Navdata.state

#Modulos para a obtencao das posicoes e angulos dos Drones
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

#Modulos para imprimir a velocidade caso precise
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

#Modulos para a obtencao da altura dos drones
def ALT1(msg):
    global H1
    H1 = msg.range


def ALT2(msg):
    global H2
    H2 = msg.range


def ALT3(msg):
    global H3
    H3 = msg.range

#Modulo para o calculo das distancias entre os drones
def DistDrone(x1,x2,x3,y1,y2,y3):
    D12 = ((x1-x2)**2 + (y1-y2)**2)**(float(1)/2)
    
    D13 = ((x1-x3)**2 + (y1-y3)**2)**(float(1)/2)
    
    D23 = ((x3-x2)**2 + (y3-y2)**2)**(float(1)/2)
    
    return (D12, D13, D23)

#Modulo para o controle autonomo
def Auto_Control(x, y, theta1, Partida1, Destino1):
    xa = 0
    ya = 0
    za = 0
    tha = 0
    d = 0
    xi, yi = Partida1
    xf, yf = Destino1

    inc_x = xf - x
    inc_y = yf - y
    angle_to_goal = atan2(inc_y, inc_x)

    d = abs((yf - yi)*x + (xi - xf)*y + (xf*yi - xi*yf)) / \
            ((yi-yf)**2 + (xf-xi)**2)**(float(1)/2)

    if angle_to_goal - theta1 > 0.1:
        tha = 0.5
    elif angle_to_goal - theta1 < -0.1:
        tha = -0.5
    else:
        tha = 0
        # relacao do ponto a reta
        position = (xf - xi) * (y - yi) - (yf - yi) * \
            (x - xi)  # esquerda + e direita -
        
        if (d > 0.1 and position > 0):
            ya = -1
        elif d > 0.1 and position < 0:
            ya = 1
        else:
            ya = 0
            if yf - y > 0.05:
                xa = 1
            else:
                xa = 0

    return (xa, ya, tha, d)


#Modulo para o controle pelo usuario e que so funciona caso os drones estegam a mais de 2 metros de distancia
def Key_Control(keys, xa, ya, za, tha, D12, D13, D23):
    if (keys[pygame.K_w] and D12 > 2 and D13 > 2 and D23 > 2):
        x = 1
        
    elif (keys[pygame.K_s] and D12 > 2 and D13 > 2 and D23 > 2):
        x = -1
    else:
        x = xa

    if (keys[pygame.K_a] and D12 > 2 and D13 > 2 and D23 > 2):
        y = 1
    elif (keys[pygame.K_d] and D12 > 2 and D13 > 2 and D23 > 2):
        y = -1
        print("d")
    else:
        y = ya

    if (keys[pygame.K_LEFT] and D12 > 2 and D13 > 2 and D23 > 2):
        th = 1
        print("LEFT")
    elif (keys[pygame.K_RIGHT] and D12 > 2 and D13 > 2 and D23 > 2):
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


# subscrive para o sonar_height para pegar a altra dos drones
SubAL1 = rospy.Subscriber("/ardrone_1/sonar_height", Range, ALT1)
SubAL2 = rospy.Subscriber("/ardrone_2/sonar_height", Range, ALT2)
SubAL3 = rospy.Subscriber("/ardrone_3/sonar_height", Range, ALT3)

# subscrive para Navdata para pegar o status do Drone
SubND1 = rospy.Subscriber("/ardrone_1/navdata", Navdata, ReceiverND1)
SubND2 = rospy.Subscriber("/ardrone_2/navdata", Navdata, ReceiverND2)
SubND3 = rospy.Subscriber("/ardrone_3/navdata", Navdata, ReceiverND3)

# subscrive para ground_truth para pegar as posicoes do drone
SubOD1 = rospy.Subscriber("/ardrone_1/ground_truth/state", Odometry, XYT1)
SubOD2 = rospy.Subscriber("/ardrone_2/ground_truth/state", Odometry, XYT2)
SubOD3 = rospy.Subscriber("/ardrone_3/ground_truth/state", Odometry, XYT3)

# Publisher no cmd_vel para o controle dos drones
pub1 = rospy.Publisher('/ardrone_1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/ardrone_2/cmd_vel', Twist, queue_size=1)
pub3 = rospy.Publisher('/ardrone_3/cmd_vel', Twist, queue_size=1)

# Publisher no takeoff para os drones planarem
pubo1 = rospy.Publisher('/ardrone_1/takeoff', Empty, queue_size=1)
pubo2 = rospy.Publisher('/ardrone_2/takeoff', Empty, queue_size=1)
pubo3 = rospy.Publisher('/ardrone_3/takeoff', Empty, queue_size=1)

# Publisher no land para os drones pousar
publ1 = rospy.Publisher('/ardrone_1/land', Empty, queue_size=1)
publ2 = rospy.Publisher('/ardrone_2/land', Empty, queue_size=1)
publ3 = rospy.Publisher('/ardrone_3/land', Empty, queue_size=1)
vazio = Empty()

# Cria um node no servidor ros
rospy.init_node('LaSid')

# Coloca os valores da velocidade de movimentacao e rotacao
speed = rospy.get_param("~speed", 0.5)
turn = rospy.get_param("~turn", 1.0)

# Inicacao de alguns parametros
xa, xb, xc = 0 ,0 ,0
ya, yb, yc = 0 ,0 ,0
za, zb, zc = 0 ,0 ,0
tha, thb, thc = 0 ,0 ,0
d1, d2, d3 = 0, 0, 0
D12, D13, D23 = 0, 0, 0

# Define as posicoes iniciais e finais dos drones
Partida1 = (6, -8)
Destino1 = (6, 8)

Partida2 = (2, -8)
Destino2 = (2, 8)

Partida3 = (-2, -8)
Destino3 = (-2, 8)

# Linha de comando necessario para conexao com o ros
r = rospy.Rate(4)

# Inicia o pygame
pygame.init()

# Tamanho da tela
pygame.display.set_mode((800, 300))

looping = True

# Variavel responsavel qual drone sera controlado
ConDrone = 0


while looping:
    
    # Pega os valores das distancias entre os drones
    D12, D13, D23 = DistDrone(x1,x2,x3,y1,y2,y3)
    
    # Verifica os botoes apertados na tela do pygame
    for event in pygame.event.get():
        # Se apertar o botao de fechar ele fecha a tela
        if event.type == pygame.QUIT:
            looping = False

        #Verifica os botoes que foram apertados   
        if event.type == pygame.KEYDOWN:
            
            # Se o botao V o drone selecionado plana
            if event.key == pygame.K_v:
                if ConDrone == 1:
                    pubo1.publish(vazio)
                
                elif ConDrone ==  2:
                    pubo2.publish(vazio)
                
                elif ConDrone == 3:
                    pubo3.publish(vazio)    
            
            # Se apertar o botao B o drone selecionado ira pousar
            if event.key == pygame.K_b:
                if ConDrone == 1:
                    publ1.publish(vazio)
                elif ConDrone == 2:
                    publ2.publish(vazio)
                elif ConDrone == 3:
                    publ3.publish(vazio)
            
            # Se o botao de 1 a 4 for apertado muda o drone a ser controlado
            # O 4 ira servir para todos os drones planarem
            if event.key == pygame.K_1:
                ConDrone = 1
            if event.key == pygame.K_2:
                ConDrone = 2
            if event.key == pygame.K_3:
                ConDrone = 3
            if event.key == pygame.K_4:
                ConDrone = 4
    

    # Caso o botao 4 tenha sido apertado todos os drones irao planar 
    if ConDrone == 4: 
        ConDrone = 1
        pubo1.publish(vazio)
        pubo2.publish(vazio)
        pubo3.publish(vazio)

    # Pega os valores x, y , th dos drones no controle autonomo e a distancia ate o destino
    xa, ya, tha, d1 = Auto_Control(x1, y1, theta1, Partida1, Destino1)
    xb, yb, thb, d2 = Auto_Control(x2, y2, theta2, Partida2, Destino2)
    xc, yc, thc, d3 = Auto_Control(x3, y3, theta3, Partida3, Destino3)

    # Pega os botoes que estao sendo pressionados
    keys = pygame.key.get_pressed()


    # A depender do drone selecionado pega os valores de x ,y, th dos drone pelo controle de usuario
    if ConDrone == 1:
        xa, ya, za, tha = Key_Control(keys, xa, ya, za, tha, D12, D13, D23)
    elif ConDrone == 2:
        xb, yb, zb, thb = Key_Control(keys, xb, yb, zb, thb, D12, D13, D23)
    elif ConDrone == 3:
        xc, yc, zc, thc = Key_Control(keys, xc, yc, zc, thc, D12, D13, D23)


    # Bloco para os Drones esperarem uns aos outros
    if abs(  (Destino1[1] - y1) - (Destino2[1] - y2) )  > 1:
        if (Destino1[1] - y1) < (Destino2[1] - y2)  and (xa != -1):
            xa = 0
        if (Destino1[1] - y1) > (Destino2[1] - y2)  and (xb != -1):
            xb = 0
    if abs(  (Destino1[1] - y1) - (Destino3[1] - y3) )  > 1: 
        if (Destino1[1] - y1) < (Destino3[1] - y3)  and (xa != -1):
            xa = 0
        if (Destino1[1] - y1) > (Destino3[1] - y3)  and (xc != -1):
            xc = 0    
    if abs(  (Destino3[1] - y3) - (Destino2[1] - y2) )  > 1:
        if (Destino3[1] - y3) < (Destino2[1] - y2)  and (xc != -1):
            xc = 0
        if (Destino3[1] - y3) > (Destino2[1] - y2)  and (xb != -1):
            xb = 0

    # Bloco para os drones pousarem quando chegarem no destino
    if ( abs((Destino1[1] - y1) ) < 0.2 ):
        publ1.publish(vazio)
        xa, ya, za = 0, 0, 0
    if ( abs((Destino2[1] - y2))  < 0.2 ):
        publ2.publish(vazio)
        xb, yb, zb = 0, 0, 0
    if ( abs((Destino3[1] - y3))  < 0.2 ):
        publ3.publish(vazio)
        xc, yc, zc = 0, 0, 0

    # Bloco de protecao para um dos drones nao chegar mais perto que 1.5, se chega o que estive em sua rota pousa
    if (D12 < 1.5):
        if (d1 > d2):
            publ2.publish(vazio)
        else:
            publ1.publish(vazio)

    if (D13 < 1.5):
        if (d1 > d3):
            publ3.publish(vazio)
        else:
            publ1.publish(vazio)

    if (D23 < 1.5):
        if (d2 > d3):
            publ3.publish(vazio)
        else:
            publ2.publish(vazio)

    # Cria modulos de movimentacao para cada um dos drones
    twist1 = Twist()
    twist2 = Twist()
    twist3 = Twist()
    
    # Modifica os valores de movimentacao do drone 1
    twist1.linear.x = xa*speed
    twist1.linear.y = ya*speed
    twist1.linear.z = za*speed
    twist1.angular.x = 0
    twist1.angular.y = 0
    twist1.angular.z = tha*turn
    
    # Publica os valores de movimentacao do drone 1
    pub1.publish(twist1)

    # Modifica os valores de movimentacao do drone 2
    twist2.linear.x = xb*speed
    twist2.linear.y = yb*speed
    twist2.linear.z = zb*speed
    twist2.angular.x = 0
    twist2.angular.y = 0
    twist2.angular.z = thb*turn
    
    # Publica os valores de movimentacao do drone 1
    pub2.publish(twist2)

    # Modifica os valores de movimentacao do drone 3
    twist3.linear.x = xc*speed
    twist3.linear.y = yc*speed
    twist3.linear.z = zc*speed
    twist3.angular.x = 0
    twist3.angular.y = 0
    twist3.angular.z = thc*turn
    # Publica os valores de movimentacao do drone 3
    pub3.publish(twist3)

    r.sleep()
pygame.quit()
