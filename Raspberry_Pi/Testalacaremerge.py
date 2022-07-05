from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import cv2
import numpy as np
import yaml
from dataHandler import imageproc, writer
from Pico import pico
from calibration import calibration

#sleep(5)

improc = imageproc()
wr = writer('data.json')
pico = pico()
cal = calibration()

def takePic(res, save=False):
	#TURN LASER OFF
	pico.laserOff()
	pico.ledsOff()

	#TAKE PIC
	with PiCamera() as camera:
		# Create the in-memory stream
		stream = BytesIO()
		camera.resolution = res#(1280, 720)#(4056, 3040) #max res
		camera.rotation = 180
		camera.capture(stream, format='jpeg')

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		#im.save("lol.jpg")
		img = np.array(im) 
		h,  w = img.shape[:2]
		camera.close()
		if save:
			im.save("picture.jpg")
		return img

def takeDepth(res, matrix):
	#TURN LASER ON
	pico.ledsOff()
	pico.laserOn()

	#TAKE PIC FOR DEPTH
	with PiCamera() as camera:
		# Create the in-memory stream
		stream = BytesIO()
		camera.resolution = res#(1280, 720)#(4056, 3040) #max res, pretty f sick #right now the cal file is only made for this specific res
		#in the futere each res should have it's own cal file
		camera.rotation = 180
		camera.capture(stream, format='jpeg')

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		#im.save("lol.jpg")
		img = np.array(im) 
		h,  w = img.shape[:2]

		#SET DEPTH IMAGE
		improc.setImg(img)
		improc.undist(matrix)
		camera.close()

def takeColor(res, matrix):
	#TURN LASER OFF
	pico.laserOff()
	pico.ledsOn()

	#TAKE PIC FOR COLOR
	with PiCamera() as camera:
		# Create the in-memory stream
		stream = BytesIO()
		camera.resolution = res#(1280, 720)#(4056, 3040) #max res, pretty f sick #right now the cal file is only made for this specific res
		#in the futere each res should have it's own cal file
		camera.rotation = 180
		camera.capture(stream, format='jpeg')

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		img = np.array(im) 
		h,  w = img.shape[:2]

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		img = np.array(im) 
		h,  w = img.shape[:2]

		#SET COLOR IMAGE
		improc.setColor(img)
		improc.undistC(matrix)
		camera.close()



#CHECK THE CONECTION WITH PICO AND SEND THE IP
pico.ck()
sleep(0.5)

################ CALIBRATION CHEK #############
#get res from pico
r = pico.getRes()
print(r)
ang_res = pico.getAng() #200 From pico
print(ang_res)
sec = 360 / ang_res 
cal.setRes(r)

res = cal.getRes()
matrix = cal.getMatrix()

if matrix == -1:
	#pico send message
	pico.startCal()
	
	#pico wait
	while not pico.doneCal():
		sleep(0.5)

	sleep(2)
	#Take image for reference and compute the matrix
	img = takePic(res)
	cal.genMatrix(img)

matrix = cal.getMatrix()

#CHECK IF LIGHTING IS OK FOR SCANNING

l0, l1 = pico.getLight()
print(f'first light{l0}, second light{l1}')
#High == more light

if (l0+l1)/2 > 22000:
	pico.wait()

	while not pico.doneCal():
		sleep(0.5)



pico.servo(17.5)
pico.sendIp()
sleep(1)
pico.laserOff()
pico.ledsOn()

#WAIT UNTIL THE USER PRESS THE BUTTON
while not pico.start():
    sleep(0.5)

sleep(2)

#TURN OFF LCD DURING SCAN
pico.LCDOff()

takeDepth(res, matrix)

sleep(0.5)

takeColor(res, matrix)

m = improc.combineFilter(dilate=3)

#immmask = Image.fromarray(improc.getMask(m).astype('uint8'), 'RGB')
#immmask.save("maska.jpg")

line = improc.localMaxQUICK(m)

#READ THE WEIGHT SENSOR
weight = pico.getWeigt() #greutate
print(weight)
##########

wr.setHeader(improc.getRes(), weight)

wr.addData(line, 0)
#cv2.destroyAllWindows()

#####SCAN AROUND THE OBJECT#####
for i in range(1, ang_res):
	#MOVE BED
	pico.rotateBed(i*sec)
	sleep(1)
	takeDepth(res, matrix)

	takeColor(res, matrix)

	#CREATE THE MASK
	m = improc.combineFilter(dilate=3)

	#EXTRACT THE LASER POSITION ALONG WITH THE COLOR
	line = improc.localMaxQUICK(m)

	#ADD DATA TO THE JSON FILE
	wr.addData(line, i*sec)

#ROTATE BED TO THE INITIAL POSITION
pico.rotateBed(ang_res*sec) # 360 deg

print('MOVEEEEEEE UPPPPPP')

#ROTATE ARM TO 90 DEGREES
pico.moveArm(90)
sleep(10)
print('GETTTT OUUUTT')

###SCAN TOP OF THE OBJECT#####
top_res = 40 #hardcoded value, if more detail scan is required this should be increased
top_sec = 360 / top_res
for i in range (top_res, 0, -1):
	#MOVE BED
	pico.rotateBedR(i*top_sec)
	sleep(1)
	takeDepth(res, matrix)

	takeColor(res, matrix)

	#CREATE THE MASK
	m = improc.combineFilter(dilate=3)

	#EXTRACT THE LASER POSITION ALONG WITH THE COLOR
	line = improc.localMaxQUICK(m)

	#ADD DATA TO THE JSON FILE
	wr.addDataTop(line, i*top_sec)

pico.LCDOn()

#SAVE THE DATA
wr.save()

#CONVERT TO GRAYSCALE
#gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
#cv2.imwrite('/home/pi/Desktop/test5.png',img)

#sleep(1)
