import numpy as np 
import matplotlib.pyplot as plt
import math as m
from Bezier import *
import time
from scipy.optimize import minimize
from sklearn.preprocessing import normalize

track_width = 1.4
set_track_width(track_width+0.1+0.5)
set_V_A_lim(14,10,5)
cone_radius = 0.15+0.1
center_offset = track_width/2
# X_origin = 47.4
# Y_origin = 49.4
# X_cone = np.array([56,74.5,  82,82.2,58.5,39.6,54.5,  46,68.7,82.3]) - X_origin
# Y_cone = np.array([72,52.7,33.5, 3.7, 3.2,36.5,35.8,83.7,83.7,70.1]) - Y_origin
# X = np.array([47.4, 51.7, 80.5, 74.1, 88.5, 54.8,47.4]) - X_origin
# Y = np.array([49.4, 79,   53,   33.1,-1.58,-4.33,49.4]) - Y_origin
X_cone = np.array([0, 5.5, 7.0, 5.5, -1.0, -7.0, -6.5, -6.0,0])
Y_cone = np.array([0, 0, 1.8, 3.7, 3.7, 3.6, 1.8, 0, 0]) + track_width
Y = np.array([0, Y_cone[1]-center_offset, Y_cone[2], Y_cone[3]+center_offset, Y_cone[4] -center_offset, Y_cone[5]+center_offset, Y_cone[6],Y_cone[7]-center_offset, 0])
X = np.array([X_cone[0], X_cone[1], X_cone[2]+center_offset, X_cone[3], X_cone[4], X_cone[5], X_cone[6]-track_width, X_cone[7], X_cone[8]])
X_0 = np.copy(X)
Y_0 = np.copy(Y)

theta = 0/57.3
rot_X = m.cos(theta)*X - m.sin(theta)*Y
rot_Y = m.sin(theta)*X + m.cos(theta)*Y

X = rot_X
Y = rot_Y
slope = generate_slopes(X,Y)
slope_0 = np.copy(slope)


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
state_radius[0] = 30 # +/- degrees of slope
state_radius[1] = center_offset - cone_radius # +/- meters offset for waypoint
state_radius[2] = 2*m.pi

tracker = []

list_k = []
list_dk = []
list_d2k = []

def total_cost(state):
    global slope
    cost = np.zeros_like(slope)
    n = len(X)
    global X_0
    global Y_0
    global slope_0
    X_ = state[n:2*n]*np.cos(state[2*n:3*n]) + X_0
    Y_ = state[n:2*n]*np.sin(state[2*n:3*n]) + Y_0 #new X,Y slope
    slope_ = state[:n] + slope_0
    total_cost = np.sum(net_cost(X_,Y_,slope_))
    tracker.append(total_cost)
    return total_cost


b = []
b.append((0,state_radius[0]))
b.append((0,state_radius[1]))
b.append((0,state_radius[2]))

bnds = []
for j in range(3):
    for i in range(len(X)):
        bnds.append(b[j])

bnds = tuple(bnds)
x0 = state.flatten()
W = np.array([1/0.06308,   1/0.18079,   1/0.038584, 1/10.647723798100904, 10, 1])# last 2 terms are weightages for continuity
set_normalization_weights(W)
Q = np.array([1,0,0,0,0,1,1]) # last 3 terms are weightages for continuity
set_term_weights(Q) 
res = minimize(total_cost, x0, bounds = bnds, method='SLSQP')
x0 = res.x
state = x0.reshape((3,len(X)))

X = state[1,:]*np.cos(state[2,:]) + X_0
Y = state[1,:]*np.sin(state[2,:]) + Y_0 #new X,Y slope
slope = state[0,:] + slope_0
print(state[0,:])

bx,by =  get_bezier_track(X,Y,slope)

plt.figure()
plt.plot(np.arange(0,len(tracker),1),tracker,label='cost')
plt.show(block=False)

plt.figure()
plt.axis('equal')
plt.xlabel('meters')
plt.ylabel('meters')
fig = plt.gcf()
ax = fig.gca()
for i in range(len(X)):
    circle = plt.Circle((X_0[i],Y_0[i]),center_offset,color='b', fill=False)
    ax.add_artist(circle)
for i in range(len(X_cone)):
    circle = plt.Circle((X_cone[i],Y_cone[i]),cone_radius,color='r',fill=True,label = 'cones')
    ax.add_artist(circle)
plt.scatter(X,Y,label='optimized waypoint')
plt.scatter(X_0,Y_0,label='original (blue circle is the waypoint region')
plt.plot(bx,by,label='expected trajectory')
plt.legend()

theta = 25/57.3
rot_X = m.cos(theta)*X - m.sin(theta)*Y
rot_Y = m.sin(theta)*X + m.cos(theta)*Y

plt.show(block=True)

X = rot_X
Y = rot_Y
slope += theta*57.3
np.fmod(slope,360.0)

wp = []
for i in range(len(X)):
    wp.append([X[i],Y[i],slope[i]])
wp = np.array(wp)
np.save('LUCIFER_WP_cones.npy',wp)
# plt.show()
