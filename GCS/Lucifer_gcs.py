import LUCIFER_COMS 
from LUCIFER_COMS import *
import time
import os
import traceback
import matplotlib.pyplot as plt
plt.ion()
fig = plt.figure()
import tkinter as tk
from tkinter import *
import math as m
# os.system("python Lucy.py")

BAUD = 230400
com = com_handler()

OFFSET_ID = 0x0001
COMMAND_ID = 0X0002
START_ID = 0x00FE 

WP_ID = 0x0005
STATE_ID = 0x0006
CALIB_ID = 0x000A
CLEAR_ID = 0x0008
SET_ORIGIN_ID = 0x000B
REC_ID_1 = 0x000C
REC_ID_0 = 0x00FC
REC_DEBUG_ID_1 = 0x000D
REC_DEBUG_ID_0 = 0x00FD

GYRO_CAL = 0x10
ACCEL_CAL = 0x20
MAG_CAL = 0x30

ERROR_CODE = 0xFF

Tx_MODE = 0x01 #default starting mode 
Tx_ID = STATE_ID #default message ID 
Tx_msg_len = 28 #default message length
rec = False
rec_vid = False

print("LUCIFER ver0.0.1")

connection = False
run = True
base = 'COM'
for i in range(5):
	name = base + str(i)
	try:
		com._init_(name,BAUD,64)#last parameter is max bytes
		print('COM_FOUND',i)
		connection = True
	except:
		pass

if(connection == False):
	print("please connect the transceiver and try again. closing.")
	run = False

file_name = 'LUCIFER_offsets_0.npy'
waypoint_file = 'LUCIFER_WP_cones.npy'#'LUCIFER_WP_slopes.npy'
log_file = 'LUCIFER_log_0.npy'
saved = False

if os.path.isfile(file_name):
	saved = True

class CAR():
	def _init_(self):
		self.X = None
		self.Y = None
		self.speed = None
		self.heading = None
		self.roll = None
		self.pitch = None
		self.acceleration = None
		self.positionAcc = None
		self.sensorHealth = None
		self.lat = []
		self.lon = []
		self.gps_lat = []
		self.gps_lon = []
		self.MODE = 0
		self.Calib = False

car = CAR()
car._init_()
waypoint_list = []
received_wp_list = []
point_count=0
waypoint_length = 0
try:
	waypoint_list = np.load(waypoint_file)
except:
	pass

latlon = np.array([0])


def send_heartbeat(car):
	global Tx_MODE
	global Tx_ID
	global Tx_msg_len
	global point_count
	global waypoint_length
	message_id = np.array([START_ID,Tx_msg_len,Tx_ID,Tx_MODE],dtype = 'int16') 
	if(Tx_ID==WP_ID or point_count>0):
		# print(Tx_ID,point_count)
		# print("sending waypoint!")
		message_id = np.array([START_ID,waypoint_length,WP_ID,Tx_MODE],dtype = 'int16') 
		n = waypoint_length - point_count
		X = int(waypoint_list[n][0]*1e2)
		Y = int(waypoint_list[n][1]*1e2)
		slope = int(waypoint_list[n][2]*1e2)
		message = list(np.array([X,Y,slope,n],dtype='int16'))
		print(message)
		message_id = np.concatenate((message_id,message),axis=0)
	if(point_count<0 and Tx_ID == WP_ID):
		# set_standby()
		Tx_ID = STATE_ID
	com.send(message_id)
	if(Tx_MODE == 0x07): #special case
		set_standby()
	# print("got here")

def handle_car_status(status):
	global Tx_ID
	if (status&0x0002==1) and (rec_vid==False):
		Tx_ID = REC_ID_0
	elif (status&0x0002==0) and (rec_vid==True):
		Tx_ID = REC_ID_1
	if (status&0x0001==1) and (rec==False):
		Tx_ID = REC_DEBUG_ID_0
	elif (status&0x0001==0) and (rec==True):
		Tx_ID = REC_DEBUG_ID_1

def readSerial():
	global Tx_MODE
	global Tx_ID
	global Tx_msg_len
	global saved
	time.sleep(0.01)
	try:
		num_bytes = com.check_recv()
		if(num_bytes):
			time.sleep(0.01)#let all of it in
			num_bytes = com.check_recv()
			message = com.read(num_bytes)

			START_SIGN = message[0]
			LENGTH = message[1]
			ID = message[2]
			car.MODE = message[3]

			if ID == OFFSET_ID:
				print('offset message')
				if(num_bytes==8): 
					#car is asking for offsets
					if os.path.isfile(file_name):
						print('File exists, loading..')
						offsets = list(np.load(file_name))
						print(offsets)
						message_id = np.array([START_SIGN,28,OFFSET_ID,car.MODE],dtype = 'int16')
						new_message = np.concatenate((message_id,offsets),axis=0) 
						com.send(new_message)
						saved = True
					else:
						print('file does not exist. Please caliberate the car and save the offsets')
				
				if(num_bytes>8 and saved == False):
					#car is sending new offsets
					print("new offsets received")
					offsets = message[4:17]
					print(offsets)
					np.save(file_name,offsets)
					saved = True

				else:
					print('car is sending offsets for no real reason other than to show off.')
					offsets = message[4:17]
					print(offsets)

			if ID == WP_ID:
				#the car is sending waypoints. store them.
				global point_count
				print('waypoint message')
				buf = np.frombuffer(message[4:],dtype='int16')
				received_wp_list.append(buf)
				n = waypoint_length-point_count
				check_wp = waypoint_list[n]
				if(m.fabs(buf[0] - check_wp[0]*100) > 2 or m.fabs(buf[1]-check_wp[1]*100)>2 or buf[3]!=n):
					print("error",buf[:4],waypoint_list[n],n)
				else:
					print("got wp")
					point_count -= 1


			if ID == STATE_ID:
				print('state message',num_bytes)
				buf = np.frombuffer(message[4:],dtype = 'int32')
				car.X =            1e-7*buf[0]
				car.Y =            1e-7*buf[1]
				dummy_lon = 	   1e-7*buf[2]
				dummy_lat = 	   1e-7*buf[3]
				car.speed =        1e-2*buf[4]
				car.heading =      1e-2*buf[5]
				car.pitch = 	   1e-2*buf[6]
				car.roll = 		   1e-2*buf[7]
				acceleration = 	   1e-2*buf[8]
				opError = 		   1e-3*buf[9]
				pError = 		   1e-3*buf[10]
				head_error = 	   1e-3*buf[11]
				Vel_Error = 	   1e-3*buf[12]
				Exec_time = 	   buf[13]
				car_status = 	   (buf[14]&0x0000FFFF)
				Hdop =			   1e-3*((buf[14]&0xFFFF0000)>>16)

				handle_car_status(car_status)

				gcs.MODE.configure(text = 'MODE = {}'.format(str(car.MODE) ) )
				gcs.latitude.configure(text = 'filtered Y = {} meters'.format(str(round(car.Y,7) ) ) )
				gcs.longitude.configure(text = 'filtered X = {} meters'.format(str(round(car.X,7) ) ) )
				gcs.gps_latitude.configure(text = 'gps latitude = {} degrees'.format(str(round(dummy_lat,7) ) ) )
				gcs.gps_longitude.configure(text = 'gps longitude = {} degrees'.format(str(round(dummy_lon,7) ) ) )				
				gcs.speed.configure(text = 'speed = {} m/s'.format(str(round(car.speed,2) ) ) )
				gcs.heading.configure(text = 'heading = {} degrees from east'.format(str(round(car.heading,2) ) ) )
				gcs.roll.configure(text = 'roll = {} degrees'.format(str( round(car.roll,3) ) ) )
				gcs.pitch.configure(text = 'pitch = {} degrees'.format(str(round(car.pitch,3) ) ) )
				gcs.acceleration.configure(text = 'acceleration = {} m/s^2'.format(str( round(acceleration,2) ) ) )
				gcs.opError.configure(text = 'op_Error = {} m'.format(str( round(opError,5) ) ) )
				gcs.pError.configure(text = 'positionError = {} m'.format(str( round(pError,2) ) ) )
				gcs.head_error.configure(text = 'heading_Error = {} degrees'.format(str( round(head_error,2) ) ) )
				gcs.Vel_Error.configure(text = 'Velocity_Error = {} m/s'.format(str( round(Vel_Error,2) ) ) )
				gcs.Exec_time.configure(text = 'max_exec_time = {} microseconds'.format(str( round(Exec_time,2) ) ) )
				gcs.Hdop.configure(text = 'gps Hdop = {} meters'.format(str( round(Hdop,2) ) ) )

				lat = np.array(car.lat)
				lon = np.array(car.lon)
				dum_lat = np.array(car.gps_lat)
				dum_lon = np.array(car.gps_lon)

				if(rec):
					# car.lon.append(car.X)
					# car.lat.append(car.Y)
					# car.gps_lon.append(dummy_lon)
					# car.gps_lat.append(dummy_lat)
					# data = np.array([car.X,car.Y,car.speed,car.heading,dummy_lon,dummy_lat,acceleration,opError,pError,head_Error,Vel_Error,Exec_time,Hdop])
					data = np.array([car.X,car.Y,dummy_lon,dummy_lat,car.speed,car.heading,car.pitch,car.roll,acceleration,Vel_Error,opError,pError,Hdop,Exec_time])
					car.lon.append(data)
					plt.scatter(car.X,car.Y)
					plt.show()
				else:
					if(len(car.lon)):
						a = time.localtime(time.time())
						# log_file = 'LUCIFER_log_filt_{}_{}_{}_{}_{}.npy'.format(a.tm_year,a.tm_mon,a.tm_mday,a.tm_hour,a.tm_min)
						# points =  np.concatenate((lat,lon),axis=0)
						# np.save( log_file, points)
						# log_file = 'LUCIFER_log_gps_{}_{}_{}_{}_{}.npy'.format(a.tm_year,a.tm_mon,a.tm_mday,a.tm_hour,a.tm_min)
						# points =  np.concatenate((dum_lat,dum_lon),axis=0)
						# np.save( log_file, points)
						log_file = 'LUCIFER_log_{}_{}_{}_{}_{}.npy'.format(a.tm_year,a.tm_mon,a.tm_mday,a.tm_hour,a.tm_min)
						np.save( log_file, car.lon)
						car.lon = []#reset
						car.lat = []
						car.gps_lat = []
						car.gps_lon = []
						plt.clf() # clear the points

				send_heartbeat(car)
				# print(Tx_MODE)
				Tx_ID = STATE_ID #reset to state ID. I don't want it to continuously register waypoints


			if ID == GYRO_CAL:
				print("just leave the car stationary. The LED will blink once when the process begins, twice when it ends. The process occurs twice.")

			if ID == ACCEL_CAL:
				print("place the car on a roughly horizontal surface, wait for the led to blink twice, then thrice, then rotate the car 180 degrees within 5 seconds, the process repeats. ")

			if ID == MAG_CAL:
				print("Do the magnetometer caliberation dance")
				print("the main LED will blink once. Point the nose of the car in the NS direction, rotate the car around the lateral axis of the car for ~8 seconds,")
				print("point the nose of the car in the EW direction, rotate the car around the longitudenal axis of the car until you see the LED blink twice")

			if ID == ERROR_CODE:
				print("An error occured. Debug it please")
		else:
			try:
				async_data = np.load("data_share.npy")
				if(async_data==1):
					set_control_check()
					np.save("data_share.npy",np.array([0]))
			except:
				pass



	except KeyboardInterrupt:
		com.close()
		exit()
	
	except Exception as e:
		print(traceback.format_exc())

	except:
		pass

	gcs.root.after(1,readSerial)


DISPLAY_WIDTH  = 800
DISPLAY_HEIGHT = 800

BACKGROUND_COLOR = 'white'

def set_manual():
	global Tx_MODE
	Tx_MODE = 0x02

def set_partial():
	global Tx_MODE
	Tx_MODE = 0x03

def set_standby():
	global Tx_MODE
	Tx_MODE = 0x01

def set_auto():
	global Tx_MODE
	Tx_MODE = 0x04

def set_666():
	global Tx_MODE
	Tx_MODE = 0x05

def set_stop():
	global Tx_MODE
	Tx_MODE = 0x00

def set_control_check():
	global Tx_MODE
	Tx_MODE = 0x07

def set_origin():
	global Tx_ID
	Tx_ID = SET_ORIGIN_ID

def mark():
	global waypoint_list
	global car
	point = np.array([car.X,car.Y,car.heading])
	waypoint_list.append(point)

def save_wp():
	global waypoint_list
	global point_count
	global waypoint_length
	waypoint_list = np.array(waypoint_list)
	point_count = len(waypoint_list)
	waypoint_length = point_count
	np.save(waypoint_file,waypoint_list)

def check_fidelity():
	global waypoint_list
	global received_wp_list
	received_wp_list = np.array(received_wp_list)
	for i in range(len(waypoint_list)):
		if waypoint_list[i][0] != received_wp_list[i][0]:
			return
	set_control_check()

def clear_gui():
	global waypoint_list
	waypoint_list = []

def clear_car():
	global Tx_ID
	Tx_ID = CLEAR_ID

def send_wp():
	global Tx_ID
	save_wp()
	Tx_ID = WP_ID

def calib():
	global Tx_ID
	global saved 
	saved = False
	Tx_ID = CALIB_ID

def record():
	global rec
	global Tx_ID
	rec = True
	Tx_ID = REC_DEBUG_ID_1

def stop_recording():
	global rec
	global Tx_ID
	rec = False
	Tx_ID = REC_DEBUG_ID_0

def record_video():
	global Tx_ID
	global rec_vid
	rec_vid = True
	Tx_ID = REC_ID_1

def stop_video():
	global Tx_ID
	global rec_vid
	rec_vid = False
	Tx_ID = REC_ID_0


class GCS():

    def __init__(self):
        
        self.root = tk.Tk()
        self.root.configure(bg=BACKGROUND_COLOR)
        self.root.resizable(False, False)
        self.root.title('Lucifer Ground Control Station')
        left = (self.root.winfo_screenwidth() - DISPLAY_WIDTH) / 2
        top = (self.root.winfo_screenheight() - DISPLAY_HEIGHT) / 2
        self.root.geometry('%dx%d+%d+%d' % (DISPLAY_WIDTH, DISPLAY_HEIGHT, left, top))
        self.frame = tk.Frame(self.root)
        self.frame.pack()
        # self.canvas = tk.Canvas(self.root, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, background='black')
        # self.canvas.pack()
        self.root.after(100, readSerial)
        
        self.latitude = tk.Label(self.frame,text='')
        self.latitude.pack()
        self.longitude = tk.Label(self.frame,text='')
        self.longitude.pack()
        self.gps_latitude = tk.Label(self.frame,text='')
        self.gps_latitude.pack()
        self.gps_longitude = tk.Label(self.frame,text='')
        self.gps_longitude.pack()
        self.speed = tk.Label(self.frame,text='')
        self.speed.pack()
        self.heading = tk.Label(self.frame,text='')
        self.heading.pack()
        self.pitch = tk.Label(self.frame,text='')
        self.pitch.pack()
        self.roll = tk.Label(self.frame,text='')
        self.roll.pack()
        self.MODE = tk.Label(self.frame,text='')
        self.MODE.pack()
        self.acceleration = tk.Label(self.frame,text='')
        self.acceleration.pack()
        self.opError = tk.Label(self.frame,text='')
        self.opError.pack()
        self.pError = tk.Label(self.frame,text='')
        self.pError.pack()
        self.head_error = tk.Label(self.frame,text='')
        self.head_error.pack()
        self.Vel_Error = tk.Label(self.frame,text='')
        self.Vel_Error.pack()
        self.Exec_time = tk.Label(self.frame,text='')
        self.Exec_time.pack()
        self.Hdop = tk.Label(self.frame,text='')
        self.Hdop.pack()

        self.manual_button = tk.Button(self.frame, text = 'MANUAL', command = set_manual)
        self.manual_button.pack()
        self.manual_P_button = tk.Button(self.frame, text = 'PARTIAL', command = set_partial)
        self.manual_P_button.pack()
        self.standby_button = tk.Button(self.frame, text = 'STANDBY', command = set_standby)
        self.standby_button.pack()
        self.cruise_button = tk.Button(self.frame, text = 'AUTO', command = set_auto)
        self.cruise_button.pack()
        self.ludicrous_button = tk.Button(self.frame, text = 'LUDICROUS', command = set_666)
        self.ludicrous_button.pack()
        self.control_check_button = tk.Button(self.frame, text = 'CONTROL_CHECK', command = set_control_check)
        self.control_check_button.pack()
        self.mark_button = tk.Button(self.frame, text = 'MARK', command = mark)
        self.mark_button.pack()
        self.mark_button = tk.Button(self.frame, text = 'SAVE_N_SEND_WP', command = send_wp)
        self.mark_button.pack()
        self.mark_button = tk.Button(self.frame, text = 'CLEAR_GUI_WP', command = clear_gui)
        self.mark_button.pack()
        self.mark_button = tk.Button(self.frame, text = 'CLEAR_CAR_WP', command = clear_car)
        self.mark_button.pack()
        self.mark_button = tk.Button(self.frame, text = 'SET ORIGIN', command = set_origin)
        self.mark_button.pack()
        self.stop_button = tk.Button(self.frame, text = 'STOP', command = set_stop)
        self.stop_button.pack()
        self.record_button = tk.Button(self.frame, text = 'RECORD', command = record)
        self.record_button.pack()
        self.record_stop_button = tk.Button(self.frame, text = 'STOP_RECORDING', command = stop_recording)
        self.record_stop_button.pack()
        self.record_button = tk.Button(self.frame, text = 'RECORD_VIDEO', command = record_video)
        self.record_button.pack()
        self.record_stop_button = tk.Button(self.frame, text = 'STOP_VIDEO', command = stop_video)
        self.record_stop_button.pack()
        self.caliberate_button = tk.Button(self.frame, text = 'CALIB A/G', command = calib)
        self.caliberate_button.pack()

    def _add_button(self, label, parent, callback, disabled=True):
        button = tk.Button(parent, text=label, command=callback)
        button.pack(side=tk.LEFT)
        button.config(state = 'disabled' if disabled else 'normal')
        return button


gcs = GCS()
tk.mainloop()




