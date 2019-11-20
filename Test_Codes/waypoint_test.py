import numpy as np 
import matplotlib.pyplot as plt
import math as m
from Bezier import *
import time

RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3
# X = np.array([0, 3, 6, 3, 1,-1,  1, 2, 4, 2, 1,-0.5,-1.4,-2,-2.7,-4,-4.5,-6.5,-3,0])
# Y = np.array([0, 0, 3, 6, 6,4.5, 3, 3, 2, 1, 1, 1.4, 2.5, 4, 5.4, 6, 6.0, 3.0, 0,0])
# X =  1.5*np.array([0, 3, 6, 4, 0,-4, -6,-3,0])
# Y = 1.25*np.array([0, 0, 3, 6, 2, 6,  3, 0,0])
X = np.array([0, 3, 6, 1, -1, 2, 4, 2,-3,-6,-3,0])
Y = np.array([0, 0, 4, 6,4.5, 3, 2, 1, 5.5, 4, 0,0])


theta = 0/57.3
rot_X = m.cos(theta)*X - m.sin(theta)*Y
rot_Y = m.sin(theta)*X + m.cos(theta)*Y

X = rot_X
Y = rot_Y
slope = generate_slopes(X,Y)
print(slope)
cost = np.zeros_like(slope)
last_cost = np.zeros_like(slope)
delta = np.zeros_like(slope)
feedback = np.ones_like(slope)
n = len(X)
init = False
N = 1000
tracker = np.zeros(N)
error = np.zeros_like(cost)
last_error = np.zeros_like(cost)
s_error = np.zeros_like(error)
d_error = np.zeros_like(error)

def total_cost(X,Y,slope):
	cost = np.zeros_like(slope)
	n = len(X)
	for i in range(len(slope)-1):
		s = i%n
		f = (i+1)%n
		cost[s] = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])
	total_cost = np.sum(cost)
	return total_cost

def local_cost(X,Y,slope,i):
	cost = np.zeros(2)
	n = len(X)
	s = (i-1)%n
	f = i%n
	cost[0] = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])
	s = f
	f = (i+1)%n
	# print(s,f)
	cost[1] = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])
	local_cost = np.sum(cost)
	return local_cost

kp = 1
ki = 1
kd = 1

for _ in range(N):
	for i in range(len(slope)-1):
		s = i%n
		f = (i+1)%n
		cost[s] = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])
	for i in range(1,len(slope)-1):
		dummy = slope[i]
		base = local_cost(X,Y,slope,i)
		slope[i] += 1
		new = local_cost(X,Y,slope,i)
		slope[i] = dummy
		delta[i] = new - base#dy/dx 
	tracker[_] = base
	for i in range(len(slope)):
		error[i] = cost[i]*delta[i]
		d_error[i] = error[i] - last_error[i]
		s_error[i] += error[i]/N
		last_error[i] = error[i]
		PID = kp*error[i] + ki*s_error[i] + kd*d_error[i]		
		slope[i] -= PID
		slope[0] = 0
		slope[-1] = 0

bx,by =  get_bezier_track(X,Y,slope)
print(slope)

# data = np.load('LUCIFER_log_2019_10_18_8_47.npy')
# coords = data[:,0:4]
# filtered = coords[:,0:2]
# lon = filtered[:,0]
# lat = filtered[:,1]

# plt.plot(np.arange(0,len(tracker),1),tracker,label='cost')
plt.xlim(-10,20)
plt.axis('equal')
plt.scatter(rot_X,rot_Y)
# plt.plot((lon[:] - lon[0]),(lat[:] - lat[0]),label='actual')
plt.plot(bx,by,label='expected')
plt.legend()

# wp = []
# for i in range(len(X)):
# 	wp.append([X[i],Y[i],slope[i]])
# wp = np.array(wp)
# np.save('LUCIFER_WP_slopes.npy',wp)

plt.show()
