#!/usr/bin/env python
#Adaptado de: https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/cameracalib.py

import numpy, cv2, glob

#Parametros
Linhas = 9
Colunas = 6
dimension = 28 #- mm
t = 0

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)

# Preparacao dos pontos do objeto usado na calibracao
objp = numpy.zeros((Linhas*Colunas, 3), numpy.float32)
objp[:,:2] = numpy.mgrid[0:Colunas, 0:Linhas].T.reshape(-1, 2)

objpoints = [] # Pontos 3D do mundo real
imgpoints = [] # Pontos 2D do plano da imagem

# Abrindo e armazenando as imagens
OpenImg = "Fotos/*.jpg"
images = glob.glob(OpenImg)

if len(images) < 9:
    print("Quantidade de imagens insuficientes. Sao necessarias pelo menos 9 imagens para realizar a calibracao!")    

else:
    nPatternFound = 0
    imgRuins = images[1]

    for fname in images:
        # t += 1

        if 'calibresult' in fname: continue
        
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        print("Lendo imagem: ", fname)

        # Encontrando cantos do tabuleiro
        ret, corners = cv2.findChessboardCorners(gray, (Colunas,Linhas), None)

        # Se forem encontrados pontos, adicione-os
        if ret == True:
            print ("Cantos encontrados, Enter para aceitar imagem, Esc para ignonar")
            # Para evitar falhas na deteccao, a funcao de cantos a nivel de subpixel e utilizada
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

            # Desenhar bordas na tela
            cv2.drawChessboardCorners(img, (Colunas,Linhas), corners2, ret)
            cv2.imshow('img', img)
            # cv2.imwrite('img{}.jpg'.format(t), img)

            k = cv2.waitKey(0) & 0xFF
            if k == 27:
                print "Imagem recusada"
                imgRuins = fname
                continue

            print("Imagem aceita")
            nPatternFound += 1
            objpoints.append(objp)
            imgpoints.append(corners2)

        else:
            imgRuins = fname

    cv2.destroyAllWindows()
    if (nPatternFound > 1):
        print("Existem {} imagens boas".format(nPatternFound))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        print("Matriz de calibracao:")
        print(mtx)
        print("Distorcao:")
        print(dist)

        #--------- Save result
        numpy.savetxt("MatrizCamera.txt", mtx, delimiter=' ')
        numpy.savetxt("DistorcaoCamera.txt", dist, delimiter=' ')

        ErroMedio = 0
        for i in xrange(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            erro = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            ErroMedio += erro

        TotalErro = ErroMedio/len(objpoints)
        print("Erro total: {}".format(TotalErro)) 

    else:
        print("Quantidade de imagens insuficientes. Sao necessarias pelo menos 9 imagens para realizar a calibracao!")