import serial
import serial.tools.list_ports
import sys
from threading import *
import time

#download serial: (sudo) pip install --upgrade pyserial


class Listener(Thread):
	def __init__(self, arm_module):
		Thread.__init__(self)
		self.daemon = True
		self.arm = arm_module

	def run(self):
		while True:
			try:
				output = self.arm.ser.read(128)
				print(output)
				if 'b' in output:
					self.arm.wait_flag = 1
			except:
				pass

class Arm(Thread):
	def __init__(self):
		Thread.__init__(self)
		self.daemon = True

		self.ser 			= serial.Serial()
		self.ser.baudrate 	= 9600
		self.ser.bytesize 	= 8
		self.ser.parity 	= 'N'
		self.ser.stopbits 	= 1
		self.ser.timeout 	= 1

		self.wait_flag		= 0

	def run(self):
		self.listener = Listener(self)
		#self.connect()
		self.listener.start()

		while True:
			self.input = raw_input()
			if   self.input == "C":
				self.connect()
			elif self.input == "V":
				self.raiseArm()
			elif self.input == "B":
				self.waitForArm()
			elif self.input == "N":
				self.lowerArm()
			else:
				self.ser.write(self.input)
			#self.waitForArm()

	def connect(self):
		ports = list(serial.tools.list_ports.comports())

		for p in ports:
			try:
				print("Port info: "),
				print p
				if 'VID:PID=1A86:7523' in p[2]:
					self.ser.port = p[0]
					self.ser.open()
					time.sleep(3)
					self.ser.write('t')
					output = self.ser.read(128)
					print("Received: %s" % output)
					if "k" in output:
						print("Connected successfully")
					return True
			except:
				print(sys.exc_info())
		return False

	def raiseArm(self):
		self.ser.write('a')

	def lowerArm(self):
		self.ser.write('h')

	def waitForArm(self):
		while True:
			output = self.ser.read(128)
			if "b" in output:
			#if self.wait_flag:
				self.wait_flag = 0
				break

#if __name__ == "__main__":
#	arm = Arm()
#	arm.run()
