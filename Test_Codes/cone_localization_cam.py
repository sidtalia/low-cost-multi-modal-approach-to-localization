import numpy as np 
import matplotlib.pyplot as plt
import math as m
from Bezier import *
import time
from scipy import stats
import cv2

RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3

track_width = 1.2
set_track_width(track_width)
cone_radius = 0.2
center_offset = track_width/2
X_cone = np.array([0, 5.5, 7.0, 5.5, -1.0, -7.0, -6.5, -6.0])
Y_cone = np.array([0, 0, 1.829, 3.658, 3.427, 3.658, 1.829, 0]) + track_width
# X = np.array([0, 3.0205, 4.8495+center_offset, 3.0205, 0.1085,-3.0205,-3.0205,0])
X_det = np.flip(np.array([0, 5.5, 7.0, 5.5, 5.5, -1.0, -7.0, -6.5, -6.0]),axis=0) #last point not present!
Y_det = np.flip(np.array([0, 0, 1.829, 3.658, 3.0, 3.427, 3.658, 1.829, 0]) + track_width,axis=0)
X_det += np.random.normal(0,0.1,len(X_det))
Y_det += np.random.normal(0,0.1,len(X_det))

def wrap360(a,b):
	diff = a - b
	if(diff>180):
		diff -= 360
	if(diff< -180):
		diff += 360
	return diff

def inrange(a,b,delta):
	if(m.fabs(wrap360(a,b))>delta):
		return False
	return True

def distancecalcy(y1,y2,x1,x2):
	delX = (x2-x1);
	delY = (y2-y1);
	delX *= delX;
	delY *= delY;
	return m.sqrt(delX + delY);   

def simulate(pos_x, pos_y, head, fov, X_det,Y_det, noise_x, noise_y):
	X_seen = []
	Y_seen = []
	for i in range(len(X_det)):
		slope = RAD2DEG*m.atan2((Y_det[i]-pos_y),(X_det[i]-pos_x))
		if(inrange(slope,head,fov/2)):
			X_seen.append(X_det[i]+noise_x)
			Y_seen.append(Y_det[i]+noise_y)
	return np.array(X_seen), np.array(Y_seen)

def shift_array(a,shift):
	arr = np.copy(a)
	for i in range(len(arr)):
		arr[i] += shift
	return arr # in c++ you won't have to return the array

def mean(arr,n):
	dummmy = 0
	for i in range(n):
		dummmy += arr[i]
	return dummmy/n

def sim_cam(X_cone, Y_cone, X_pos, Y_pos, head):
	X = X_cone - X_pos 
	Y = Y_cone - Y_pos

	X_cone_ref = X*m.sin(head) - Y*m.cos(head)
	Y_cone_ref = (X*m.cos(head) + Y*m.sin(head)) - 0.24

	image = cv2.imread("coneslam.png")
	Xmax = image.shape[1]//2
	Ymax = image.shape[0]//2
	X_Center = image.shape[1]//2
	Y_Center = image.shape[0]//2

	fov_h = 60/57.3
	K_h = m.tan(fov_h/2)
	fov_v = fov_h*120/160
	K_v = m.tan(fov_v/2)
	H = 0.11

	radius = 5
	thickness = -1
	color = (255,0,0)
	xpix = np.zeros_like(X_cone_ref)
	ypix = np.zeros_like(xpix)

	for i in range(len(X_cone_ref)):
		ypix[i] = int(Y_Center + int(H*Ymax/(Y_cone_ref[i]*K_v)))
		xpix[i] = int(X_Center + int(X_cone_ref[i]*Xmax/(Y_cone_ref[i]*K_h)))
		# print(int(xpix[i]), int(ypix[i]))
		center_coordinates = (int(xpix[i]),int(ypix[i]))
		image = cv2.circle(image, center_coordinates, radius, color, thickness)
	
	cv2.imshow('img',image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

#using ICP inspired method for now
def find_transform(X_proj, Y_proj, x_det, y_det):
	x_proj = np.copy(X_proj)
	y_proj = np.copy(Y_proj)
	n = len(x_proj)
	k = len(x_det)
	size = n*k
	x_tf = np.zeros(size)
	y_tf = np.zeros(size)
	error_radius = 1
	for k in range(5):
		counter = 0
		mean_x = 0
		mean_y = 0
		for i in range(len(x_proj)):
			for j in range(len(x_det)):
				dx = x_det[j] - x_proj[i]
				dy = y_det[j] - y_proj[i]
				if( m.sqrt(dx**2 + dy**2) > error_radius/(k+1)):
					dx = 100
					dy = 100
				else:
					x_tf[n*i+j]= dx
					y_tf[n*i+j]= dy
					mean_x += dx
					mean_y += dy
					counter+=1
		shift_x = mean_x/counter
		shift_y = mean_y/counter
		x_proj = shift_array(x_proj, shift_x)
		y_proj = shift_array(y_proj, shift_y)
	final_shift_x = mean(x_proj - X_proj,len(X_proj))
	final_shift_y = mean(y_proj - Y_proj,len(X_proj))
	return final_shift_x, final_shift_y


pos_x = 0.0# true position
pos_y = 0.0 # true position
head = 0

Error_pos_x = 0.5
Error_pos_y = 0.5

x_cone_det, y_cone_det = simulate(pos_x,pos_y,head,60,X_det,Y_det,Error_pos_x,Error_pos_y)
x_cone_proj,y_cone_proj = simulate(pos_x,pos_y,head,60,X_cone,Y_cone,0,0)

t = time.time()
x_tf, y_tf = find_transform(x_cone_proj, y_cone_proj, x_cone_det, y_cone_det)
dt = time.time()-t
print('dt = ',dt*1000,' ms')
print(x_tf,y_tf)
# true_x, true_y = localize(x_cone_sim, y_cone_sim, X_cone, Y_cone)
x_corrected, y_corrected = simulate(pos_x,pos_y,head,90,X_cone,Y_cone,x_tf,y_tf)

plt.axis('equal')
plt.arrow(pos_x,pos_y,m.cos(head*DEG2RAD),m.sin(head*DEG2RAD),head_width=0.1)
plt.scatter(x_cone_proj, y_cone_proj,label='projected cone locations')
plt.scatter(x_cone_det, y_cone_det, label='cones detected')
plt.scatter(x_corrected,y_corrected, label='corrected')
sim_cam(x_cone_proj,y_cone_proj,pos_x,pos_y,head/57.3)
# plt.scatter(X_cone+x_tf, Y_cone+y_tf, label = 'projected cones after correction')
# plt.scatter(x_tf,y_tf)
plt.legend()
plt.show()





