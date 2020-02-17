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

def pixel_aggregation(cut_img):
	hsv = cv2.cvtColor(cut_img, cv2.COLOR_BGR2HSV) # Color space conversion, BRG to HSV

	dark_red_a = numpy.array([0, 100, 100])  # Range dark red
	dark_red_b = numpy.array([10, 255, 255])
	
	light_red_a = numpy.array([160, 200, 100])  # Range light red
	light_red_b = numpy.array([179, 255, 255])
	
	mask_dark = cv2.inRange(hsv, dark_red_a, dark_red_b)  # Mask dark red
	mask_light = cv2.inRange(hsv, light_red_a, light_red_b)  # Mask light red

	mask = mask_dark + mask_light

	structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))  # Structuring element
	open_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, structure)  # Morphological operator
	result = cv2.bitwise_and(cut_img, cut_img, mask=open_mask) # I calculate the pixel-by-pixel conjunction

    #------------------------------ Thereshold
	gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
	ret, threshold = cv2.threshold(numpy.absolute(gray), 50, 255, cv2.THRESH_BINARY)
	
	return threshold

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
    # result = pixel_aggregation(cropped_img)
    
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