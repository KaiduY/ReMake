import time
import serial
import netifaces

class pico:
	"""This class handle the serial communication with the Raspberry Pi Pico via UART.
	"""
	
	def __init__(self, dr=0):
		"""An interface for the microcontroller.

		Args:
			dr (int, optional): normal direction for the rotating bed. Defaults to 0.
		"""

		self.ser = serial.Serial(            
			port='/dev/ttyS0',
			baudrate = 115200,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=1
		)
		
		self.dr = dr
		self.drarm = 0
	
	def sendM(self, mes):
		"""Tries to send a message over serial and waits for the confirmation.

		Args:
			mes (str): message to be send
		"""

		try:
			self.ser.write(mes.encode())
			time.sleep(0.03)
			received_data = self.ser.read()              #read serial port
			time.sleep(0.03)
			data_left = self.ser.inWaiting()             #check for remaining byte
			received_data += self.ser.read(data_left)
			
			if received_data:
				received_data = received_data.decode()
				
			if received_data == 'ok\n':
				self.ser.reset_input_buffer()

			else:
				time.sleep(0.1)
				received_data = self.ser.read()              #read serial port
				time.sleep(0.03)
				data_left = self.ser.inWaiting()             #check for remaining byte
				received_data += self.ser.read(data_left)
				
				if received_data:
					received_data = received_data.decode()
					
				if received_data == 'ok\n':
					self.ser.reset_input_buffer()
					
				else:
					print('Confimation not received, sending again the message')
					self.sendM(mes)
				
				
		except: 
			print("errror 53")
			
	def sendM_wc(self, mes):
		"""Tries to send a message over serial without the need of a confirmation.

		Args:
			mes (str): message to be send
		"""

		try:
			self.ser.write(mes.encode())
			time.sleep(0.03)
				
		except OSError as error : 
			print(error)
			
	def readM(self):
		"""Tries to read the serial buffer and reconstruct the message received.

		Returns:
			str: message received
		"""

		try:
			received_data = self.ser.read()              #read serial port
			time.sleep(0.03)
			data_left = self.ser.inWaiting()             #check for remaining byte
			received_data += self.ser.read(data_left)
			return received_data.decode("utf-8", "ignore")
		except:
			print("error 71")
	
	def get_interfaces(self):
		"""Get local networking interfaces

		Returns:
			interface: local interface
		"""

		interfaces = netifaces.interfaces()
		interfaces.remove('lo')

		out_interfaces = dict()

		for interface in interfaces:
			addrs = netifaces.ifaddresses(interface)
			out_addrs = dict()
			if netifaces.AF_INET in addrs.keys():
				out_addrs["ipv4"] = addrs[netifaces.AF_INET]
			if netifaces.AF_INET6 in addrs.keys():
				out_addrs["ipv6"] = addrs[netifaces.AF_INET6]
			out_interfaces[interface] = out_addrs

		return out_interfaces 
		
	def sendIp(self):
		"""Send the local IP to the microcontroller.
		"""

		ip = self.get_interfaces()['wlan0']['ipv4'][0]['addr']
		self.sendM('i'+ip+'\n')
	
	def rotateBed(self, angle):
		"""Rotate the bed to a specific angle.

		Args:
			angle (float): angle to rotate to
		"""

		mes = 'r' + str(self.dr)+'_'+str(angle) + '\n'
		self.sendM(mes)
	
	def rotateBedR(self, angle):
		"""Rotate the bed in reverse to a specific angle.

		Args:
			angle (float): angle to rotate to
		"""

		if self.dr == 1:
			dr = 0
		elif self.dr == 0:
			dr = 1
		mes = 'r' + str(dr)+'_'+str(angle) + '\n'
		self.sendM(mes)
	
	def moveArm(self, angle):
		"""Rotate the arm to a specific angle.

		Args:
			angle (float): angle to rotate to
		"""

		mes = 'z' + str(self.drarm)+'_'+str(angle) + '\n'
		self.sendM(mes)
	
	def servo(self, angle):
		"""Rotate the servo holding the laser to a specific angle.

		Args:
			angle (float): angle to rotate to
		"""

		mes = 'k' + str(angle) + '\n'
		self.sendM(mes)
		time.sleep(2)
	
	def laserOn(self):
		"""Turn on the laser.
		"""

		mes = 'l0\n'
		self.sendM(mes)
	
	def laserOff(self):
		"""Turn off the laser.
		"""

		mes = 'l1\n'
		self.sendM(mes)
	
	def ledsOn(self):
		"""Turn on the LEDs.
		"""

		mes = 'e0\n'
		self.sendM(mes)
	
	def ledsOff(self):
		"""Turn off the LEDs.
		"""

		mes = 'e1\n'
		self.sendM(mes)
	
	def LCDOn(self):
		"""Turn on the LCD backlight.
		"""

		mes = 'x1\n'
		self.sendM(mes)
	
	def LCDOff(self):
		"""Turn off the LCD backlight.
		"""

		mes = 'x0\n'
		self.sendM(mes)
	
	def getWeigt(self):
		"""Read the pressure sensor and get the weigt of the object scanned.

		Returns:
			float: weight
		"""

		mes = 'w\n'
		self.sendM(mes)
		time.sleep(0.2)
		
		a=1 #to be found out
		b=0
			
		weight = int(self.readM()) * a + b
		return weight
	
	def getLight(self):
		"""Read the light sensors.

		Returns:
			list: the light intensity for each sensor
		"""

		mes = 'p\n'
		self.sendM(mes)
		time.sleep(0.2)
		rx = self.readM()
		while not rx.strip():
			rx = self.readM()
			time.sleep(0.1)
		rcv = rx.split('_')
		l0, l1 = int(rcv[0]), int(rcv[1])
		return (l0, l1)
	
	def getRes(self):
		"""Get the resolution set in by the user.

		Returns:
			int: selected resolution 
		"""

		mes = 'o\n'
		self.sendM(mes)
		time.sleep(0.2)
		rx = self.readM()
		while not rx.strip():
			rx = self.readM()
			time.sleep(0.1)
		res = int(rx)
		return res
	
	def getAng(self):
		"""Get the numer of samples for the scan set in by the user.

		Returns:
			int: number of samples
		"""

		mes = 'y\n'
		self.sendM(mes)
		time.sleep(0.2)
		rx = self.readM()
		while not rx.strip():
			rx = self.readM()
			time.sleep(0.1)
		res = int(rx)
		return res
	
	def startCal(self):
		"""Starts the calibration process.
		"""

		mes = 'h\n'
		self.sendM(mes)
		
	def ck(self):
		"""Verify the connection.
		"""

		mes = 'c\n'
		self.sendM(mes)
	
	def doneCal(self):
		"""Check if the previous operation is completed.

		Returns:
			bool: True if the operation is completed, False otherwise
		"""

		mes = self.readM()
		if mes == 'start\n':
			return True
		return False
	
	def wait(self):
		"""Tell the user that the exterior light is too bright.
		"""

		mes = 'j\n'
		self.sendM(mes)
	
	def start(self):
		"""Get the input from the user to check if the scan should start.

		Returns:
			bool: True if the scan should start, False otherwise
		"""
		
		mes = 's\n'
		self.sendM(mes)
		time.sleep(0.1)
		
		mes = self.readM()
		if mes == 'start\n':
			return True
		return False
	
	

"""
pico = pico()
pico.ck()
time.sleep(1)
pico.sendIp()
pico.rotateBed(90)
pico.laserOff()

pico.ledsOn()

pico.servo(10)

pico.servo(90)

#pico.LCDOff()
print(pico.getWeigt())
#pico.sendM(mes)

while not pico.start():
    #print(pico.readM())
    time.sleep(0.5)
print('LES GOOO')
    
"""
