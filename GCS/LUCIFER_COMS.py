import serial
import numpy as np

class com_handler():
	def _init_(self,COM,BAUD,number_of_bytes):
		self.BAUD = BAUD
		self.COM = COM
		self.number_of_bytes = number_of_bytes
		self.ser =  serial.Serial(self.COM, self.BAUD, timeout=0)# com port, baud rate, timeout.
		if self.ser.isOpen():
				self.ser.close()
		self.ser.open()

	def close(self):
		self.ser.close()
		self.ser.close()

	def clear(self):
		try:
			self.read(64)
		except:
			pass

	def send(self,info_Tx):
		message_Tx = info_Tx.tobytes()
		self.ser.write(message_Tx)

	def check_recv(self):
		return self.ser.inWaiting()
	
	def read(self,number_of_bytes):
		message_Rx = self.ser.read(number_of_bytes)
		info_Rx = np.frombuffer(message_Rx,dtype = 'int16' )
		self.clear()# this is to make sure that the data is always new. 
		return info_Rx