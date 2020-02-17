# -*- coding: utf-8 -*-
#-------------------------
# Author: Walber C de Jesus Rocha
# University: Universidade Federal do Recôncavo da Bahia - UFRB
# Paper title: Sistema de medição de distância baseado em visão computacional utilizando laser de linha
# Project: Construção de uma Colônia de Robôs Autônomos para Reconhecimento, Busca e Inspeção
#-------------------------

import cv2
import numpy
from math import pi, tan, atan2, degrees

def cut_image(img):
    cut_img = img[240:340, 0:640].copy()
    cut_img = cv2.normalize(cut_img, cut_img, 0, 255, cv2.NORM_MINMAX)
    cut_img = cv2.GaussianBlur(cut_img, (1, 1), 0)
    return cut_img

def gradient_sobel(cut_img):
    # ------------------------------ Gradient Sobel
    gray = cv2.cvtColor(cut_img, cv2.COLOR_BGR2GRAY)
    sobel_gy = cv2.Sobel(gray, cv2.CV_8U, 0, 1, ksize=3)

    # ------------------------------ Thereshold
    ret, threshold = cv2.threshold(numpy.absolute(
        sobel_gy), 50, 255, cv2.THRESH_BINARY)
    kernel = numpy.ones((2, 2), numpy.uint8)
    erosion = cv2.erode(threshold, kernel, iterations=1)

    return erosion

# ------------------------------ HoughLines
def filter_image(threshold):
    y = []
    lines = cv2.HoughLinesP(threshold, 1, numpy.pi/2, 20, 30, 15)

    if lines is None:
        return None
    else:
        for line in lines:
            for x1, y1, x2, y2 in line:
                y.extend([y1, y2])

    return sorted(set(y))

# ------------------------------ Distance calculate
def get_distance(y):
    if(y is None):
        return None
    else:
        yFinal = []
        distFinal = []
        yFinal.append(y[0])

        for i in range(1, len(y)):
            yAux = y[i-1]

            if(abs(y[i] - yAux) >= 2):
                yFinal.append(y[i])

        for i in yFinal:
            DPF = abs((i + 240) - (height/2))
            theta = RPP * DPF + DR
            dist = laser_height/tan(theta)
            distFinal.append(round(dist, 3))

        return sorted(distFinal, key=float)

def laser_RangeFinder(img):
    cropped_img = cut_image(img)
    result = gradient_sobel(cropped_img)
    y_coordinates = filter_image(result)
    distances = get_distance(y_coordinates)

    return(print(distances))

if __name__ == '__main__':

    RPP = 0.001620713896608  # Radians per pixel
    DR = -0.024015443752004  # Radial displacement
    laser_height = 2.8  # laser height (cm)

    # Image path
    img = cv2.imread('Images_Tests/50cm.jpg')
    
    width = numpy.size(img, 1)  # get width image
    height = numpy.size(img, 0)  # get height image

    laser_RangeFinder(img)