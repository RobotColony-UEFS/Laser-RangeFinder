#!/usr/bin/env python
import rospy, cv2, numpy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Distancia: " + str(data.data) + "cm" )

def Medidas():
    rospy.init_node('Medidas', anonymous=True)
    rospy.Subscriber('Distancias', String, callback)
    print("OK")
    rospy.spin()

if __name__ == '__main__':
    Medidas()
