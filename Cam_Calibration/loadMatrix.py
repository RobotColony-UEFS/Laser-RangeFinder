#!/usr/bin/env python
import numpy
from math import atan2, degrees, atan
import cv2

img = cv2.imread('base/Angulo/D/45graus - D.jpg')

PI = []
PE = []

matriz = open('Calibracao/MatrizCamera.txt', 'r')
aux = []
aux = matriz.readlines()
for i in range(len(aux)):
	PI.append(aux[i].split())

matriz2 = open('Calibracao/DistorcaoCamera.txt', 'r')
aux2 = []
aux2 = matriz2.readlines()
for i in range(len(aux2)):
	PE.append(aux2[i].split())

PI = numpy.asarray(PI, dtype=numpy.float, order='C')
PE = numpy.asarray(PE, dtype=numpy.float, order='C')

teste = cv2.undistortPoints(img, PI, PE)