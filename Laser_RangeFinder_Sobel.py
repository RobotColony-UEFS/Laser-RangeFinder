# -*- coding: utf-8 -*-
import cv2, numpy
from math import pi, tan, atan2, degrees

def fitImage(img):
    CorteImg = img[240:340, 0:640].copy()
    CorteImg = cv2.normalize(CorteImg, CorteImg, 0, 255, cv2.NORM_MINMAX)
    CorteImg = cv2.GaussianBlur(CorteImg, (1, 1), 0)
    return CorteImg

# ------------------------------ Gradiente de Sobel
def gradientSobel(CorteImg):
    cinza = cv2.cvtColor(CorteImg, cv2.COLOR_BGR2GRAY)
    gy = cv2.Sobel(cinza, cv2.CV_8U, 0, 1, ksize=3)

    #------------------------------ Thereshold
    ret, threshold = cv2.threshold(numpy.absolute(gy), 50, 255, cv2.THRESH_BINARY)

    kernel = numpy.ones((2, 2), numpy.uint8)
    erosion = cv2.erode(threshold, kernel, iterations=1)

    return erosion

# ------------------------------ HoughLines
def filterImg(threshold):
    y = []
    linhas = cv2.HoughLinesP(threshold, 1, numpy.pi/2, 20, 30, 15)

    if linhas is None:
        return None
    else:
        for linha in linhas:
            for x1, y1, x2, y2 in linha:
                y.extend([y1, y2])

    return sorted(set(y))

# ------------------------------ Calculo da distancia
def distance(y, RPP, DR, alturaLaser):
    if(y is None):
        return None

    else:
        yFinal = []
        distFinal = []
        yFinal.append(y[0])
        
        for i in range(1, len(y)):
            yAuxiliar = y[i-1]

            if(abs(y[i] - yAuxiliar) >= 2):
                yFinal.append(y[i])

        for i in yFinal:
            DPF = abs((i + 240) - (altura / 2))
            theta = RPP * DPF + DR
            dist = alturaLaser/tan(theta)
            distFinal.append(round(dist, 3))

        return sorted(distFinal, key=float)

if __name__ == '__main__':

    RPP = 0.001620713896608  # Radianos por pixel
    DR = -0.024015443752004  # Deslocamento radiano
    alturaLaser = 2.8  # Altura do laser(cm)
    
    img = cv2.imread('imgTest/40cm.jpg')
    largura = numpy.size(img, 1)  # Largura
    altura = numpy.size(img, 0)  # Altura

    def laser_RangeFinder(img):
        ImgCorte = fitImage(img)
        threshold = gradientSobel(ImgCorte)
        y = filterImg(threshold)
        distancias = distance(y, RPP, DR, alturaLaser)

        return(print(distancias))

    laser_RangeFinder(img)