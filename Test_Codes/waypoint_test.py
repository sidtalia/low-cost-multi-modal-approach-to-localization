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
# X = np.array([0, 3, 6, 1, -1, 2, 4, 2,-3,-6,-3,0])
# Y = np.array([0, 0, 4, 6,4.5, 3, 2, 1, 5.5, 4, 0,0])
track_width = 1.2
set_track_width(track_width)
cone_radius = 0
center_offset = track_width/2
X_cone = np.array([0, 3.0205, 4.8495, 3.0205, 0.1085,-3.0205,-3.0205,0])
Y_cone = np.array([0, 0, 1.829, 3.658, 3.247, 4.272, 0, 0]) + track_width
# X = np.array([0, 3.0205, 4.8495+center_offset, 3.0205, 0.1085,-3.0205,-3.0205,0])
Y = np.array([0, center_offset, 1.829 + track_width, 3.658+track_width+center_offset, 3.247+ track_width -center_offset, 4.272+track_width+center_offset, center_offset, 0])
X = np.array([X_cone[0], X_cone[1], X_cone[2]+center_offset, X_cone[3], X_cone[4], X_cone[5], X_cone[6], X_cone[7]])
# X = np.array([0, 3, 6, 1, -1, 2, 4, 2,-3,-6,-3,0])
# Y = np.array([0, 0, 4, 6,4.5, 3, 2, 1, 5.5, 4, 0,0])

X_0 = np.copy(X)
Y_0 = np.copy(Y)

theta = 0/57.3
rot_X = m.cos(theta)*X - m.sin(theta)*Y
rot_Y = m.sin(theta)*X + m.cos(theta)*Y

X = rot_X
Y = rot_Y
slope = generate_slopes(X,Y)


dims = 3
state = np.zeros((dims,len(slope)))
# set the state:
R = np.zeros_like(slope)
theta = np.zeros_like(slope)
state[0,:] = slope
state[1,:] = R
state[2,:] = theta
# set state origins
state_0 = np.copy(state) # create a copy that will be used later as reference
# set state radius
state_radius = np.zeros(dims)
state_radius[0] = 60 # +/- degrees of slope
state_radius[1] = center_offset # +/- meters offset for waypoint
state_radius[2] = 2*m.pi
# change in state used for evaluating partial derivative:
delta = np.zeros(dims)
delta[0] = 0.01 # 1 degree
delta[1] = 0.001 # 0.01 meters shift
delta[2] = 0.001 # 0.01 meters shift
# change in state due to change in variable:
delta_state = np.zeros_like(state)
# cost of each section
cost = np.zeros_like(slope)
last_cost = np.zeros_like(slope)
n = len(X)
N = 1000
tracker = np.zeros(N)
error = np.zeros_like(state)
last_error = np.zeros_like(state)
s_error = np.zeros_like(state)
d_error = np.zeros_like(state)


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
	local_cost = m.fabs(cost[1]-cost[0])
	return local_cost

kp = 1/N
ki = 0/N
kd = 0/N

def check(x,origin,delta):
	if(x > origin + delta):
		x = origin + delta
		return x
	if(x < origin - delta):
		x = origin - delta
		return x	
	return x

min_cost=1e3
min_cost_state = np.copy(state)

for _ in range(N):
	for i in range(len(slope)-1):
		s = i%n
		f = (i+1)%n
		cost[s] = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])

	for i in range(1,len(slope)-1):
		for k in range(dims):
			dummy = state[k][i]
			base = local_cost(X,Y,state[0,:],i) #X,Y,slope
			state[k][i] += delta[k]
			X_ = state[1,:]*np.cos(state[2,:]) + X_0
			Y_ = state[1,:]*np.sin(state[2,:]) + Y_0 #new X,Y slope
			new = local_cost(X_,Y_,state[0,:],i)
			state[k][i] = dummy
			delta_state[k][i] = (new - base)/delta[k]#dy/dx 

	tracker[_] = np.sum(cost)
	if(tracker[_]<min_cost):
		min_cost_state = state
		min_cost = tracker[_]

	for i in range(len(slope)):
		for k in range(dims):	
			error[k][i] = cost[i]*delta_state[k][i]
			d_error[k][i] = error[k][i] - last_error[k][i]
			s_error[k][i] += error[k][i]/N
			last_error[k][i] = error[k][i]
			PID = kp*error[k][i] + ki*s_error[k][i] + kd*d_error[k][i]		
			state[k][i] -= PID
			state[k][i] = check(state[k][i],state_0[k][i],state_radius[k])
			state[k][0] = 0
			state[k][-1] = 0

	# state[1][-3] = state_0[1][-3]+center_offset/2
	# state[2][-3] = state_0[2][-3]-center_offset/2
	slope = state[0,:]
	X = state[1,:]*np.cos(state[2,:]) + X_0
	Y = state[1,:]*np.sin(state[2,:]) + Y_0 #new X,Y slope

# print(min_cost_state)
state = np.copy(min_cost_state)
slope = state[0,:]
X = state[1,:]*np.cos(state[2,:]) + X_0
Y = state[1,:]*np.sin(state[2,:]) + Y_0 #new X,Y slope
bx,by =  get_bezier_track(X,Y,slope)
print(slope)

# data = np.load('LUCIFER_log_2019_10_18_8_47.npy')
# coords = data[:,0:4]
# filtered = coords[:,0:2]
# lon = filtered[:,0]
# lat = filtered[:,1]
plt.figure()
plt.plot(np.arange(0,len(tracker),1),tracker,label='cost')
plt.show(block=False)

plt.figure()
plt.axis('equal')
# plt.scatter(rot_X,rot_Y)
# plt.plot(rot_X,rot_Y)
plt.scatter(X_cone,Y_cone,color='orange',label = 'cones')
plt.scatter(X,Y,label='waypoint')
# plt.scatter(state_0[1,:],state_0[2,:])
# plt.plot((lon[:] - lon[0]),(lat[:] - lat[0]),label='actual')
plt.plot(bx,by,label='expected trajectory')
plt.legend()
plt.show(block=False)

# wp = []
# for i in range(len(X)):
# 	wp.append([X[i],Y[i],slope[i]])
# wp = np.array(wp)
# np.save('LUCIFER_WP_slopes.npy',wp)

plt.show()
