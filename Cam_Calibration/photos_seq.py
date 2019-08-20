from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.start_preview()
t = 1;

while(t != 16):
	sleep(3)
	camera.capture('/home/pi/rangerFinder/Calibracao/img{}.jpg'.format(t))
	t = t+1
	print("Imagem {} salva".format(t))
	
camera.stop_preview()
