import numpy as np 
import matplotlib.pyplot as plt
import math as m
from Bezier import *

RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3
# X = np.array([0, 3, 6, 3, 1,-1,  1, 2, 4, 2, 1,-0.5,-1.4,-2,-2.7,-4,-4.5,-6.5,-3,0])
# Y = np.array([0, 0, 3, 6, 6,4.5, 3, 3, 2, 1, 1, 1.4, 2.5, 4, 5.4, 6, 6.0, 3.0, 0,0])
X =  1.5*np.array([0, 3, 6, 4, 0,-4, -6,-3,0])
Y = 1.25*np.array([0, 0, 3, 6, 2, 6,  3, 0,0])

theta = 30/57.3
rot_X = m.cos(theta)*X - m.sin(theta)*Y
rot_Y = m.sin(theta)*X + m.cos(theta)*Y

X = rot_X
Y = rot_Y
slope = generate_slopes(X,Y)
cost = np.zeros_like(slope)
last_cost = np.zeros_like(slope)
delta = np.zeros_like(slope)
feedback = np.ones_like(slope)
n = len(X)
init = False
tracker = np.zeros(100)

for _ in range(100):
	for i in range(len(slope)-1):
		s = i%n
		f = (i+1)%n
		last_cost[s]=cost[s]
		cost[s] = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])
	delta = cost - last_cost
	for i in range(len(feedback)):
		feedback[i] = delta[i]
	# print(feedback)
	if not init:
		slope = slope-0.1
		init = True
	else:
		slope[1] = slope[1] + np.sign(feedback[0])*m.exp(1-_/10)
		slope[-2] = slope[-2] + 0.1
		# slope[1:-1] = slope[1:] + np.sign(feedback[:-1])*m.exp(1-_/10)
		slope[0] = theta*57.3
		# slope[1] = slope[0]
		# slope[-2] = slope[0]
		tracker[_] = feedback[-2]
	print(feedback)
# print(np.sum(cost))#18.99
plt.plot(np.arange(10,20,0.1),tracker)



bx,by =  get_bezier_track(X,Y,slope)

# data = np.load('LUCIFER_log_2019_10_18_8_47.npy')
# coords = data[:,0:4]
# filtered = coords[:,0:2]
# lon = filtered[:,0]
# lat = filtered[:,1]

# plt.axis('equal')
# plt.plot(rot_X,rot_Y)
# plt.plot((lon[:] - lon[0]),(lat[:] - lat[0]),label='actual')
# plt.plot(bx,by,label='expected')
plt.legend()

# wp = []
# for i in range(len(X)):
# 	wp.append([X[i],Y[i],slope[i]])
# wp = np.array(wp)
# np.save('LUCIFER_WP_slopes.npy',wp)

plt.show()
