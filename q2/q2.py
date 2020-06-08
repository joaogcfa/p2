#!/usr/bin/python
# -*- coding: utf-8 -*-

# Este NÃO é um programa ROS

from __future__ import print_function, division 

import cv2
import os,sys, os.path
import numpy as np

print("Rodando Python versão ", sys.version)
print("OpenCV versão: ", cv2.__version__)
print("Diretório de trabalho: ", os.getcwd())

# Arquivos necessários
video = "logomarca.mp4"

# Cria o detector BRISK
brisk = cv2.BRISK_create()


# Configura o algoritmo de casamento de features que vê *como* o objeto que deve ser encontrado aparece na imagem
bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# Define o mínimo de pontos similares
MINIMO_SEMELHANCAS = 18

def find_good_matches(descriptor_image1, frame_gray):
    """
        Recebe o descritor da imagem a procurar e um frame da cena, e devolve os keypoints e os good matches
    """

    frame_rgb = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2RGB)
    des1 = descriptor_image1
    kp2, des2 = brisk.detectAndCompute(frame_rgb,None)

    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = bf.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)

    return kp2, good



if __name__ == "__main__":



    # Inicializa a aquisição da webcam
    cap = cv2.VideoCapture(video)

    original_rgb = cv2.imread("pomba_red.png")  # Imagem a procurar
    img_original = cv2.cvtColor(original_rgb, cv2.COLOR_BGR2GRAY)

    kp1, des1 = brisk.detectAndCompute(img_original ,None)

    print("Se a janela com a imagem não aparecer em primeiro plano dê Alt-Tab")


    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret == False:
            #print("Codigo de retorno FALSO - problema para capturar o frame")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
            #sys.exit(0)

        frame_rgb = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp2, good_matches = find_good_matches(des1, gray)

        if len(good_matches) > MINIMO_SEMELHANCAS:
            img3 = cv2.drawMatches(original_rgb,kp1,frame_rgb,kp2, good_matches, None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            cv2.imshow('BRISK features', img3)
        else:
            cv2.imshow("BRISK features", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


