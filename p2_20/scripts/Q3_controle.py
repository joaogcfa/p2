#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_empty_world.launch 


from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations


x = None
y = None

contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos = np.degrees(transformations.euler_from_quaternion(lista))    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1



virado_y = False
cheguei = False
lado = 2

def go_to(x_objetivo,y_objetivo):

    x_positivo = True
    y_positivo = True
    virado_y = False
    # cheguei = False
    # lado = 2
    if x_objetivo < 0:
        x_positivo = False
    if y_objetivo < 0:
        y_positivo = False

    while x_objetivo > 0 and x < x_objetivo and virado_y == False and x_positivo == True:
        vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)
        print("ESTOU INDO NO X")
        rospy.sleep(0.1)
        

    if x > x_objetivo and virado_y == False:
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
        velocidade_saida.publish(vel)
        rospy.sleep(0.2)
        if y_objetivo > 0:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
            velocidade_saida.publish(vel)
            rospy.sleep((math.pi/2)/0.1)
            virado_y = True
        if y_objetivo < 0:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
            velocidade_saida.publish(vel)
            rospy.sleep((math.pi/2)/0.1)
            virado_y = True
    
    if virado_y == True:
        while y_objetivo > 0 and y < y_objetivo:
            vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
            velocidade_saida.publish(vel)
            print("ESTOU INDO NO Y")
            rospy.sleep(0.1)
        if y > y_objetivo:
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            velocidade_saida.publish(vel)
            rospy.sleep(0.2)

if __name__=="__main__":

    rospy.init_node("q3")

    


    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    tolerancia = 0.15

    



    rospy.sleep(1.0) # contorna bugs de timing    

    while not rospy.is_shutdown():

            

        
        go_to(1,1)



            
        rospy.sleep(0.1)    


# if cheguei == True:
#             vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
#             velocidade_saida.publish(vel)
#             rospy.sleep((lado/2)/0.1)
#             vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
#             velocidade_saida.publish(vel)
#             rospy.sleep(0.1)
#             vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
#             velocidade_saida.publish(vel)
#             rospy.sleep((math.pi/3)/0.1)