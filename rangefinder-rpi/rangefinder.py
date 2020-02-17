#code: utf-8
# -------------------------
# Autor: Walber C de Jesus Rocha
# Universidade: Universidade Federal do Recôncavo da Bahia - UFRB
# Título do Trabalho: Sistema de medição de distância baseado em visão computacional utilizando laser de line
# Projeto: Construção de uma Colônia de Robôs Autônomos para Reconhecimento, Busca e Inspeção
# -------------------------

#!/usr/bin/python

from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
from math import tan, pi
import RPi.GPIO as gpio
import cv2
import numpy
import smbus

# CAM PARAMS
camera = PiCamera()
width_capture = 640
heigth_capture = 480
camera.resolution = (width_capture, heigth_capture)
# camera.framerate = 15
camera.brightness = 40
rawCapture = PiRGBArray(camera, size=(width_capture, heigth_capture))
camera.start_preview()
sleep(0.1)

# GPIO PARAMS
gpio.setmode(gpio.BOARD)
gpio.setup(11, gpio.OUT)

# LUMINOSITY SENSOR
bus = smbus.SMBus(1)


def luminosity():
    data = bus.read_i2c_block_data(0x23, 0x11)  # Read sensor
    # Conversion to a decimal number
    return round(((data[1] + (256 * data[0])) / 1.2), 2)


def adjustment_img(img):
	initial_cut = (height/2)  # 240 pixel
	final_cut = (height/2 + height/4)  # 360 pixel
	cut_img = img[initial_cut:340, 0:width].copy() # Valid range: 15 ~ 100
	cut_img = cv2.normalize(cut_img, cut_img, 0, 255, cv2.NORM_MINMAX) # Normalization of the color scale
	cut_img = cv2.GaussianBlur(cut_img, (1, 1), 0)  # Blur image, noise removal
	return cut_img, initial_cut

# Identification and separation of the red line using pixel aggregation method
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

# ------------------------------ Identification and separation of the line through the Sobel Gradient
def gradient_sobel(cut_img):
    gray = cv2.cvtColor(cut_img, cv2.COLOR_BGR2GRAY)
    sobel_gy = cv2.Sobel(gray, cv2.CV_8U, 0, 1, ksize=3)

    #------------------------------ Thereshold
    ret, threshold = cv2.threshold(numpy.absolute(
        sobel_gy), 50, 255, cv2.THRESH_BINARY)
    return threshold

def filter_image(threshold):  # ------------------------------ Filter HoughLinesP
    y = []
    lines = cv2.HoughLinesP(threshold, 1, numpy.pi/2, 20, 30, 15)
    if lines is None:
        yMax = None
    else:
        for line in lines:
            for x1, y1, x2, y2 in line:
                y.extend([y1, y2])
        yMax = max(y)
    return yMax  # maximum height

# ------------------------------ Distance calculate
def get_distance(yMax, rpc, ro, laser_height):
    if(yMax == None):
        distance = 0
    else:
        pfc = abs((yMax+initial_cut)-(height/2))
        theta = rpc * pfc + ro
        distance = laser_height/tan(theta)
    return distance

if __name__ == '__main__':

    RPP = 0.001344472778584  # Radians per pixel
    DR = -0.016636285331381  # Radial displacement
    laser_height = 2.8  # laser height (cm)

    try:
        gpio.output(11, gpio.HIGH)

        for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
			img = frame.array
			width = numpy.size(img, 1)  # Width frame captured
			height = numpy.size(img, 0)  # height frame captured
			cut_img, initial_cut = adjustment_img(img)
			
			# result = gradient_sobel(cut_img)
			result = pixel_aggregation(cut_img)
			
			yMax = filter_image(result)
			distance = get_distance(yMax, RPP, DR, laser_height)
			luminosity = luminosity()
			print("Distancia: {}cm, Iluminacao: {}lux".format(round(distance, 3), luminosity))
			rawCapture.truncate(0)
	
	except KeyboardInterrupt:
		gpio.output(11, gpio.LOW)
		camera.close()