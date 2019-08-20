#!/usr/bin/env python
import cv2, numpy, smbus, rospy
from math import pi, tan, atan2, degrees
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

largura = 640
altura = 480
rpc = 0.001620713896608 # Radianos por pixel
ro = -0.024015443752004 # Deslocamento radiano
h = 2.8 #Altura do laser(cm)

pub2 = rospy.Publisher('Distancias', String, queue_size=10)

def AjusteImg(img):
	#------------------------------ Ajuste de Regiao
	corteInicial = altura/2 #240 pixel
	corteFinal = altura/2 + altura/4 #360 pixel

	CorteImg = img[corteInicial:corteFinal, 0:largura].copy() #Intervalo valido 15~100cm
	CorteImg = cv2.normalize(CorteImg, CorteImg, 0, 255, cv2.NORM_MINMAX) #Normalizacao da escala de cores
	CorteImg = cv2.GaussianBlur(CorteImg, (1,1), 0) #Desfoque na imagem, remocao de ruidos
	return CorteImg, corteInicial

def AgregPixel(CorteImg): # Identificacao e separacao da linha vermelha
    hsv = cv2.cvtColor(CorteImg, cv2.COLOR_BGR2HSV) # Conversao do espaco de cores, BRG para HSV

    vermelho1_a = numpy.array([0,100,100]) # Range vermelho escuro
    vermelho1_b = numpy.array([10,255,255])

    vermelho2_a = numpy.array([160,100,100]) # Range vermelho claro
    vermelho2_b = numpy.array([179,255,255])

    mascaraEscura = cv2.inRange(hsv, vermelho1_a, vermelho1_b) # Mascara vermelho escuro
    mascaraClara = cv2.inRange(hsv, vermelho2_a, vermelho2_b) # Mascara vermelho claro
    mascara=mascaraEscura+mascaraClara #Mascara

    estrutura = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) # Elemento estruturante
    mascaraAberta = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, estrutura) # Operador morfologico
    res = cv2.bitwise_and(CorteImg, CorteImg, mask=mascaraAberta) # Calculo a conjuncao pixel a pixel

    #------------------------------ Thereshold
    cinza = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, th1 = cv2.threshold(numpy.absolute(cinza), 50, 255, cv2.THRESH_BINARY)
    return th1

def FiltroImg(th1): #------------------------------ Filtro HoughLinesP
	y = []
	# yMax = 0
	linhas = cv2.HoughLinesP(th1, 1, numpy.pi/180, 20, 30, 15)
	if(linhas == None).all():
		yMax = None
	else:
		for linha in linhas: # Percorrendo o vetor resultante do filtro HoughLinesP
		    for x1,y1,x2,y2 in linha:
		        y.extend([y1, y2]) # Adicionando coordenadas de altura ao vetor y
		yMax = max(y)
	# print(yMax)
	return yMax # Altura maxima

def Distancia(yMax, rpc, ro, alturaLaser): #------------------------------ Calculo da distancia
	corteInicial = altura/2
	if(yMax == None):
		distancia = 0
	else:
		pfc = abs((yMax+corteInicial)-(altura/2))
		theta = rpc * pfc + ro
		distancia = alturaLaser/tan(theta)
	return distancia

def callback(data):
	br = CvBridge()
	img = br.imgmsg_to_cv2(data, "bgr8")
	
	CorteImg, corteInicial = AjusteImg(img) # Imagem ajustada
	th1 = AgregPixel(CorteImg) # Imagem contendo a linha detectada por Gradiente de Sobel
	y = FiltroImg(th1) # Altura da linha
	distancia = round(Distancia(y, rpc, ro, h), 3) # Distancia
	print(distancia)
	pub2.publish(str(distancia))

def AgregacaoPixel():
	rospy.init_node('AgregacaoPixel', anonymous=True)
	rospy.Subscriber('rangeFinder', Image, callback)
	print("OK")
	rospy.spin()

if __name__ == '__main__':
	AgregacaoPixel()
