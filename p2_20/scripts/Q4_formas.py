#! /usr/bin/env python
# -*- coding:utf-8 -*-
# Sugerimos rodar com:
# roslaunch my_simulation formas.launch

from __future__ import print_function, division
import rospy

import numpy as np

import cv2

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError


import visao_module
import math


ranges = None
minv = 0
maxv = 10

media = []
maior_contorno_area = []
centro = []
distancia = 0

bridge = CvBridge()


def scaneou(dado):
    global ranges
    global minv
    global maxv
    global distancia
    # print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    # print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
    distancia = dado.ranges[0]
 
# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global media
    global maior_contorno_area
    global centro

    # print("frame")
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")


        centro, img, resultados =  visao_module.processa(cv_image)


        media, maior_contorno_area = visao_module.identifica_cor(cv_image)



        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        variavel = 0
        # print('ex', e)

tolerancia = 20

if __name__=="__main__":

    rospy.init_node("q4")

    topico_imagem = "/camera/rgb/image_raw/compressed"
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    while not rospy.is_shutdown():
        print(maior_contorno_area)

        if len(media) != 0 and len(centro) !=0:
            if media[0] > (centro[0] + tolerancia):
                vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
                velocidade_saida.publish(vel)
            if media[0] < (centro[0] - tolerancia):
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
                velocidade_saida.publish(vel)
            if media[0] < (centro[0] + tolerancia) and media[0] > (centro[0]-tolerancia) and distancia > 1:
                vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
            if media[0] < (centro[0] + tolerancia) and media[0] > (centro[0]-tolerancia) and distancia < 1:    
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                print ("PAREI")
      
        rospy.sleep(0.1)



