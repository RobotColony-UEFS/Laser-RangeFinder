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
camera.framerate = 15
# camera.brightness = 40
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
	corteInicial = 240 #240 pixel
	corteFinal = 340 #360 pixel

	CorteImg = img[corteInicial:340, 0:largura].copy() #Intervalo valido 15~100cm
	CorteImg = cv2.normalize(CorteImg, CorteImg, 0, 255, cv2.NORM_MINMAX) #Normalizacao da escala de cores
	CorteImg = cv2.GaussianBlur(CorteImg, (1,1), 0) #Desfoque na imagem, remocao de ruidos
	return CorteImg, corteInicial

# @profile
def GradienteSobel(CorteImg): #------------------------------ Identificacao e separacao da linha atraves do Gradiente de Sobel
	cinza = cv2.cvtColor(CorteImg, cv2.COLOR_BGR2GRAY)
	gy = cv2.Sobel(cinza, cv2.CV_8U, 0, 1, ksize=3)

	#------------------------------ Thereshold
	ret, threshold = cv2.threshold(numpy.absolute(gy), 50, 255, cv2.THRESH_BINARY)
	return threshold

# @profile
def FiltroImg(threshold): #------------------------------ Filtro HoughLinesP
	y = []
	linhas = cv2.HoughLinesP(threshold, 1, numpy.pi/2, 20, 30, 15)
	if linhas is None:
		yMax = None
	else:
		for linha in linhas: # Percorrendo o vetor resultante do filtro HoughLinesP
		    for x1,y1,x2,y2 in linha:
		        y.extend([y1, y2]) # Adicionando coordenadas de altura ao vetor y
		yMax = max(y)
	return yMax # Altura maxima

# @profile
def Distancia(yMax, RPP, DR, alturaLaser): #------------------------------ Calculo da distancia
	if(yMax == None):
		distancia = 0
	else:
		DPF = abs((yMax+corteInicial)-(altura/2))
		theta = RPP * DPF + DR
		distancia = alturaLaser/tan(theta)
	return distancia


if __name__ == '__main__':

	RPP = 0.001620713896608 # Radianos por pixel
	DR = -0.024015443752004 # Deslocamento radiano
	alturaLaser = 2.8 #Altura do laser(cm)

	try:
		gpio.output(11, gpio.HIGH)

		for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
			img = frame.array

			largura = numpy.size(img, 1) # Largura
			altura = numpy.size(img, 0) # Altura

			CorteImg, corteInicial = AjusteImg(img) # Imagem ajustada
			threshold = GradienteSobel(CorteImg) # Imagem contendo a linha detectada por Gradiente de Sobel
			yMax = FiltroImg(threshold) # Altura da linha
			distancia = Distancia(yMax, RPP, DR, alturaLaser)
			iluminacao = Luminosidade()
			print("Distancia: {}cm, Iluminacao: {}lux".format(round(distancia,3), iluminacao))

			rawCapture.truncate(0)#limpeza de quadro para atualizacao do frame

	except KeyboardInterrupt:
		gpio.output(11, gpio.LOW)
		camera.close()