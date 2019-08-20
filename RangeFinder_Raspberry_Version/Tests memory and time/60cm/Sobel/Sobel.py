import cv2, numpy, sys
from math import pi, tan, atan2, degrees

@profile
def AjusteImg(img):
	#------------------------------ Ajuste de Regiao
	corteInicial = 240 #240 pixel
	corteFinal = 340 #340 pixel

	CorteImg = img[corteInicial:corteFinal, 0:largura].copy() #Intervalo valido 15~100cm
	CorteImg = cv2.normalize(CorteImg, CorteImg, 0, 255, cv2.NORM_MINMAX) #Normalizacao da escala de cores
	CorteImg = cv2.GaussianBlur(CorteImg, (1,1), 0) #Desfoque na imagem, remocao de ruidos
	return CorteImg, corteInicial

@profile
def GradienteSobel(imgCrop):
	cinza = cv2.cvtColor(CorteImg, cv2.COLOR_BGR2GRAY)
	gy = cv2.Sobel(cinza, cv2.CV_8U, 0, 1, ksize=3)

	#------------------------------ Thereshold
	ret, th1 = cv2.threshold(numpy.absolute(gy), 50, 255, cv2.THRESH_BINARY)
	return th1

@profile
def FiltroImg(th1):
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

@profile
def Distancia(yMax, rpc, ro, alturaLaser): #------------------------------ Calculo da distancia
	if(yMax == None):
		distancia = 0
	else:
		pfc = abs((yMax+corteInicial)-(altura/2))
		theta = rpc * pfc + ro
		distancia = alturaLaser/tan(theta)
	return distancia

if __name__ == '__main__':

	rpc = 0.001620713896608 # Radianos por pixel
	ro = -0.024015443752004 # Deslocamento radiano
	h = 2.8 #Altura do laser(cm)

	img = cv2.imread(sys.argv[1])
	
	largura = numpy.size(img, 1) # Largura
	altura = numpy.size(img, 0) # Altura

	CorteImg, corteInicial = AjusteImg(img) # Imagem ajustada
	th1 = GradienteSobel(CorteImg) # Imagem contendo a linha detectada por Gradiente de Sobel
	y = FiltroImg(th1) # Altura da linha
	distancia = Distancia(y, rpc, ro, h) # Distancia