#!/usr/bin/env python
import cv2, numpy, smbus, rospy
import RPi.GPIO as gpio

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

gpio.setmode(gpio.BOARD)
gpio.setup(11, gpio.OUT)

def FluxoCamera():
	pub = rospy.Publisher('rangeFinder', Image, queue_size=10)
	rospy.init_node('FluxoCamera', anonymous=True)
	rate = rospy.Rate(0.3) # 2hz
	camera = PiCamera()
 	camera.resolution = (640, 480)
	rawCapture = PiRGBArray(camera)

	time.sleep(0.1)
	br = CvBridge()

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		try:
			gpio.output(11, gpio.HIGH) #Acender laser
			img = frame.array
			image_message = br.cv2_to_imgmsg(img, "bgr8")
			rospy.loginfo("Imagem publicada")
			pub.publish(image_message)
			rawCapture.truncate(0)
			rate.sleep()

		except (KeyboardInterrupt, rospy.ROSInterruptException, SystemExit), e:
			gpio.output(11, gpio.LOW) #Desligar laser
			camera.close() #Desligar camera
			sys.exit(0)
			raise

if __name__ == '__main__':
    try:
        FluxoCamera()
    
    except (KeyboardInterrupt, rospy.ROSInterruptException, SystemExit), e:
		gpio.output(11, gpio.LOW) #Desligar laser
		camera.close() #Desligar camera
		sys.exit(0)
		pass