#!/usr/bin/env python
import cv2, numpy, smbus, rospy, time, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def Imagem():

	pub = rospy.Publisher('rangeFinder', Image, queue_size=10)
	rospy.init_node('Imagem', anonymous=True)
	img = cv2.imread(sys.argv[1])
	br = CvBridge()
	envio = br.cv2_to_imgmsg(img, "bgr8")
	rospy.loginfo("Imagem publicada")
	pub.publish(envio)

if __name__ == '__main__':
    try:
        Imagem()
    
    except (KeyboardInterrupt, rospy.ROSInterruptException, SystemExit), e:
		sys.exit(0)
		pass