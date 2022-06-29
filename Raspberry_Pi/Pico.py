import time
import serial
import netifaces

class pico:
	
	def __init__(self, dr=0):
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
		try:
			self.ser.write(mes.encode())
			time.sleep(0.03)
				
		except OSError as error : 
			print(error)
			
	def readM(self):
		try:
			received_data = self.ser.read()              #read serial port
			time.sleep(0.03)
			data_left = self.ser.inWaiting()             #check for remaining byte
			received_data += self.ser.read(data_left)
			return received_data.decode("utf-8", "ignore")
		except:
			print("error 71")
	
	def get_interfaces(self):

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
		ip = self.get_interfaces()['wlan0']['ipv4'][0]['addr']
		self.sendM('i'+ip+'\n')
	
	def rotateBed(self, angle):
		mes = 'r' + str(self.dr)+'_'+str(angle) + '\n'
		self.sendM(mes)
	
	def moveArm(self, angle):
		mes = 'z' + str(self.drarm)+'_'+str(angle) + '\n'
		self.sendM(mes)
	
	def servo(self, angle):
		mes = 'k' + str(angle) + '\n'
		self.sendM(mes)
		time.sleep(2)
	
	def laserOn(self):
		mes = 'l0\n'
		self.sendM(mes)
	
	def laserOff(self):
		mes = 'l1\n'
		self.sendM(mes)
	
	def ledsOn(self):
		mes = 'e0\n'
		self.sendM(mes)
	
	def ledsOff(self):
		mes = 'e1\n'
		self.sendM(mes)
	
	def LCDOn(self):
		mes = 'x1\n'
		self.sendM(mes)
	
	def LCDOff(self):
		mes = 'x0\n'
		self.sendM(mes)
	
	def getWeigt(self):
		mes = 'w\n'
		self.sendM(mes)
		time.sleep(0.2)
		
		a=1 #to be found out
		b=0
			
		weight = int(self.readM()) * a + b
		return weight
	
	def getLight(self):
		mes = 'p\n'
		self.sendM(mes)
		time.sleep(0.2)
		rcv = str(self.readM()).split('_')
		l0, l1 = int(rcv[0]), int(rcv[1])
		return (l0, l1)
	
	def getRes(self):
		mes = 'o\n'
		self.sendM(mes)
		time.sleep(0.2)
		res = int(self.readM())
		return res
	
	def getAng(self):
		mes = 'y\n'
		self.sendM(mes)
		time.sleep(0.2)
		res = int(self.readM())
		return res
	
	def startCal(self):
		mes = 'h\n'
		self.sendM(mes)
		
	def ck(self):
		mes = 'c\n'
		self.sendM(mes)
	
	def doneCal(self):
		mes = self.readM()
		if mes == 'start\n':
			return True
		return False
	
	def wait(self):
		mes = 'j\n'
		self.sendM(mes)
	
	def start(self):
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
