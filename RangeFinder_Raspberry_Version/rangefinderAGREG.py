#code: utf-8
#!/usr/bin/python

from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
from math import tan, pi
import RPi.GPIO as gpio
import cv2, numpy, smbus

#PARAMETROS DA CAMERA
width = 640
heigth = 480

camera = PiCamera()
camera.resolution = (width, heigth)
# camera.framerate = 15
camera.brightness = 40
rawCapture = PiRGBArray(camera, size=(width, heigth))
camera.start_preview() 
sleep(0.1) #prove o aquecimento da camera

#PARAMETROS DA GPIO
gpio.setmode(gpio.BOARD)
gpio.setup(11, gpio.OUT)

#INICIALIZACAO SENSOR LUZ
bus = smbus.SMBus(1)

def Luminosidade():
  	data = bus.read_i2c_block_data(0x23, 0x11)#leitura da porta
  	return round(((data[1] + (256 * data[0])) / 1.2), 2)#conversao para um numero decimal

def AjusteImg(img):
	#------------------------------ Ajuste de Regiao
	corteInicial = altura/2 #240 pixel
	corteFinal = altura/2 + altura/4 #360 pixel

	CorteImg = img[corteInicial:340, 0:largura].copy() #Intervalo valido 15~100cm
	CorteImg = cv2.normalize(CorteImg, CorteImg, 0, 255, cv2.NORM_MINMAX) #Normalizacao da escala de cores
	CorteImg = cv2.GaussianBlur(CorteImg, (1,1), 0) #Desfoque na imagem, remocao de ruidos
	return CorteImg, corteInicial

# @profile
def AgregacaoPixel(CorteImg): # Identificacao e separacao da linha vermelha
    hsv = cv2.cvtColor(CorteImg, cv2.COLOR_BGR2HSV) # Conversao do espaco de cores, BRG para HSV

    vermelho1_a = numpy.array([0,100,100]) # Range vermelho escuro
    vermelho1_b = numpy.array([10,255,255])

    vermelho2_a = numpy.array([160,200,100]) # Range vermelho claro
    vermelho2_b = numpy.array([179,255,255])

    mascaraEscura = cv2.inRange(hsv, vermelho1_a, vermelho1_b) # Mascara vermelho escuro
    mascaraClara = cv2.inRange(hsv, vermelho2_a, vermelho2_b) # Mascara vermelho claro
    mascara=mascaraEscura+mascaraClara #Mascara

    estrutura = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1)) # Elemento estruturante
    mascaraAberta = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, estrutura) # Operador morfologico
    res = cv2.bitwise_and(CorteImg, CorteImg, mask=mascaraAberta) # Calculo a conjuncao pixel a pixel

    #------------------------------ Thereshold
    cinza = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, th1 = cv2.threshold(numpy.absolute(cinza), 50, 255, cv2.THRESH_BINARY)
    return th1

# @profile
def GradienteSobel(CorteImg): #------------------------------ Identificacao e separacao da linha atraves do Gradiente de Sobel
	cinza = cv2.cvtColor(CorteImg, cv2.COLOR_BGR2GRAY)
	gy = cv2.Sobel(cinza, cv2.CV_8U, 0, 1, ksize=3)

	#------------------------------ Thereshold
	ret, th1 = cv2.threshold(numpy.absolute(gy), 50, 255, cv2.THRESH_BINARY)
	return th1

# @profile
def FiltroImg(th1): #------------------------------ Filtro HoughLinesP
	y = []
	linhas = cv2.HoughLinesP(th1, 1, numpy.pi/2, 20, 30, 15)
	if linhas is None:
		yMax = None
	else:
		for linha in linhas: # Percorrendo o vetor resultante do filtro HoughLinesP
		    for x1,y1,x2,y2 in linha:
		        y.extend([y1, y2]) # Adicionando coordenadas de altura ao vetor y
		yMax = max(y)
	return yMax # Altura maxima

# @profile
def Distancia(yMax, rpc, ro, alturaLaser): #------------------------------ Calculo da distancia
	if(yMax == None):
		distancia = 0
	else:
		pfc = abs((yMax+corteInicial)-(altura/2))
		theta = rpc * pfc + ro
		distancia = alturaLaser/tan(theta)
	return distancia


if __name__ == '__main__':

	RPP = 0.001344472778584
	DR = -0.016636285331381
	alturaLaser = 2.8 #Altura do laser(cm)

	try:
		gpio.output(11, gpio.HIGH)

		for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
			img = frame.array

			largura = numpy.size(img, 1) # Largura
			altura = numpy.size(img, 0) # Altura

			CorteImg, corteInicial = AjusteImg(img) # Imagem ajustada
			threshold = AgregacaoPixel(CorteImg) # Imagem contendo a linha detectada por Agregacao de pixel
			yMax = FiltroImg(threshold) # Altura da linha
			distancia = Distancia(yMax, RPP, DR, alturaLaser)
			iluminacao = Luminosidade()
			print("Distancia: {}cm, Iluminacao: {}lux".format(round(distancia,3), iluminacao))

			rawCapture.truncate(0)#limpeza de quadro para atualizacao do frame

	except KeyboardInterrupt:
		gpio.output(11, gpio.LOW)
		camera.close()