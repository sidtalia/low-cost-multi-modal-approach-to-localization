import numpy as np
import math as m
import time
RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3

track_width = 1.2
max_curvature = 0.3
ratio = 0.35

W = np.zeros(4) # doesn't matter how you initialize it.
Q = np.zeros(4)
V_lim = 10
lat_Acc_limit = 10
lon_Acc_limit = 10
C_min = 0.1

def set_track_width(tw):
	global track_width
	track_width = (tw)*5

def set_normalization_weights(weights):
	global W
	W = weights

def set_term_weights(weights):
	global Q
	Q = weights

def set_V_A_lim(Vmax,lat_A_max,lon_A_max):
	global V_lim
	V_lim = Vmax
	global lat_Acc_limit
	lat_Acc_limit = lat_A_max
	global lon_Acc_limit
	lon_Acc_limit = lon_A_max
	global C_min
	C_min = lat_Acc_limit/(V_lim**2)

def distancecalcy(y1,y2,x1,x2):
	delX = (x2-x1);
	delY = (y2-y1);
	delX *= delX;
	delY *= delY;
	return m.sqrt(delX + delY);   

def anglecalcy(x1,x2,y1,y2):
	angle = RAD2DEG*m.atan2((y2-y1),(x2-x1));
	if(angle<0):
		angle += 360;
	return angle;

def angle_difference(x1,x2,x3,y1,y2,y3):
	angle1 = anglecalcy(x1,x2,y1,y2)
	angle2 = anglecalcy(x2,x3,y2,y3)
	angle_diff = m.fabs(angle1-angle2)
	if(angle_diff>360):
		angle_diff -= 360
	return angle_diff

def wrap_360(angle):
	if(angle>360):
		angle -= 360
	if(angle<0):
		angle += 360
	return angle

def generate_slopes(X,Y):
	circuit = False
	if(distancecalcy(Y[0],Y[-1],X[0],X[-1])<1):
		circuit = True
	slope = np.empty_like(X)
	for i in range(1,len(X)-1):
		angle1 = anglecalcy( X[i-1], X[i], Y[i-1], Y[i] )
		angle2 = anglecalcy( X[i], X[i+1], Y[i], Y[i+1] )
		if(m.fabs(angle1 - angle2) > 180):
			angle1 -= 360
		slope[i] = ( angle1 + angle2 )*0.5

	if(circuit):
		angle1 = anglecalcy( X[-2], X[-1], Y[-2], Y[-1] )
		angle2 = anglecalcy( X[0], X[1], Y[0], Y[1] )
		if(m.fabs(angle1 - angle2) > 180):
			angle1 -= 360
		slope[0]  =  ( angle1 + angle2 )*0.5;
		slope[-1] = slope[0]
	else:
		slope[0] = anglecalcy( X[0], X[1], Y[0], Y[1] );
		slope[-1] = anglecalcy( X[-2], X[-1], Y[-2], Y[-1] )

	return slope

def acute_angle(A,B):
	a = m.fabs(A-B)
	while(a>180):
		a -= 180
	return a


def area(x1, y1, angle1, x2, y2, angle2):
	X = distancecalcy(y1,y2,x1,x2)
	base = anglecalcy(x1,x2,y1,y2)
	B = acute_angle(angle1,base)*DEG2RAD
	C = acute_angle(angle2,base)*DEG2RAD
	A = m.pi - (B+C)
	return m.fabs(((m.sin(B)*m.sin(C)/m.sin(A))))*X**2

def get_Intermediate_Points(slope1, slope2, X1, X2, Y1, Y2):
	global track_width
	global ratio
	int1 = np.zeros(2)
	int2 = np.zeros(2)
	d = distancecalcy(Y2,Y1,X2,X1)
	ratio = 0.4 - 0.06*(d/(track_width+d))
	# if(d>track_width):
	# 	d = track_width
	int1[0] = X1 + ratio*m.cos(slope1*DEG2RAD)*d
	int1[1] = Y1 + ratio*m.sin(slope1*DEG2RAD)*d
	int2[0] = X2 - ratio*m.cos(slope2*DEG2RAD)*d
	int2[1] = Y2 - ratio*m.sin(slope2*DEG2RAD)*d
	return int1,int2

def get_bezier(X1,X2,Y1,Y2,slope1,slope2):
	int1,int2 = get_Intermediate_Points(slope1,slope2,X1,X2,Y1,Y2)
	Px = np.array([X1,int1[0],int2[0],X2])
	Py = np.array([Y1,int1[1],int2[1],Y2])
	t = np.arange(0,1,0.05)
	T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
	Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
	By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
	return Bx,By

def arc_length(X1,Y1,X2,Y2,X3,Y3,X4,Y4):
	L1 = distancecalcy(Y1,Y2,X1,X2)
	L2 = distancecalcy(Y2,Y3,X2,X3)
	L3 = distancecalcy(Y3,Y4,X3,X4)
	L4 = distancecalcy(Y4,Y1,X4,X1)
	L = L1+L2+L3
	L = 0.5*(L+L4)
	return L

def get_T(X1,Y1,X2,Y2,X3,Y3,X4,Y4):
	L1 = distancecalcy(Y1,Y2,X1,X2)
	L2 = distancecalcy(Y2,Y3,X2,X3)
	L3 = distancecalcy(Y3,Y4,X3,X4)
	L4 = distancecalcy(Y4,Y1,X4,X1)
	L = L1+L2+L3
	L = 0.5*(L+L4)
	t1 = 0.5*(L1/(L1+L2))
	t2 = 1 - 0.5*(L3/(L3+L2))
	return np.array([t1,t2])

def Curv(t,KX1,KX2,KX3,KY1,KY2,KY3):
	delX = t*t*KX1 + t*KX2 + KX3
	delY = t*t*KY1 + t*KY2 + KY3
	del2X = 2*t*KX1 + KX2
	del2Y = 2*t*KY1 + KY2
	denominator = delX*delX + delY*delY
	dummy = denominator
	denominator *= denominator*denominator
	denominator = np.sqrt(denominator)
	del3Y = 2*KY1
	del3X = 2*KX1
	second_denominator = denominator*dummy 
	dK = ((del3Y*delX - del3X*delY)/denominator) - (3*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator)
	sub_term_1 = (delX*del2Y - del2X*delY)
	sub_term_2 = 2*(delX*del2X + delY*del2Y)
	third_denominator = np.fabs(second_denominator*dummy)
	sub_term_3 = (del3Y*delX - del3X*delY)
	sub_term_4 = 2*(del2X**2 + del2Y**2 + del3X*delX+del3Y*delY)
	sub_term_5 = - del3X*del2Y + del3Y*del2X
	term_1 = 3.75*(sub_term_1*(sub_term_2**2))/third_denominator
	term_2 = -3*(sub_term_3*sub_term_2)/second_denominator
	term_3 = -1.5*(sub_term_1*sub_term_4)/second_denominator
	term_4 = sub_term_5/denominator
	d2K = term_1 + term_2 + term_3 + term_4
	return dK,d2K

def check_range(x,i):
	if(i):
		if(x>1):
			return 1
		if(x<0.5):
			return 0.5
		return x
	if(x<0):
		return 0
	if(x>0.5):
		return 0.5
	return x

def C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3):
	delX = t*t*KX1 + t*KX2 + KX3
	delY = t*t*KY1 + t*KY2 + KY3
	del2X = 2*t*KX1 + KX2
	del2Y = 2*t*KY1 + KY2
	denominator = delX*delX + delY*delY
	dummy = denominator
	denominator *= denominator*denominator
	denominator = np.sqrt(denominator)
	Curvature = ((delX*del2Y) - (delY*del2X))
	Curvature /= denominator
	return Curvature

def V_A_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3):
	delX = t*t*KX1 + t*KX2 + KX3
	delY = t*t*KY1 + t*KY2 + KY3
	del2X = 2*t*KX1 + KX2
	del2Y = 2*t*KY1 + KY2
	return delX,delY,del2X,del2Y

def time_from_P_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3,Px,Py,C):
	global V_lim,lat_Acc_limit,lon_Acc_limit,C_min
	res = 0.01
	var = np.arange(0,1,res)
	dX,dY,d2X,d2Y = V_A_from_K_t(var,KX1,KX2,KX3,KY1,KY2,KY3)
	#bezier velocity
	Vb = np.sqrt(dX**2 + dY**2)
	tangent_i = dX/Vb# normalize
	tangent_j = dY/Vb
	#bezier lat lon acceleration (body frame) 
	Ablon = d2X*tangent_i + d2Y*tangent_j
	Ablat = d2X*tangent_j - d2Y*tangent_i# longitudenal and lateral acceleration

	index = np.argmax(C)
	t_max = np.round(t[index],2)
	C_max = C[index]
	if(np.fabs(C_max)<C_min):
		C_max = C_min
	#find min. velocity along the bez. curve
	V_min = np.sqrt(lat_Acc_limit/np.fabs(C_max))
	dXm,dYm,d2Xm,d2Ym = V_A_from_K_t(t_max,KX1,KX2,KX3,KY1,KY2,KY3)
	#find min. bez. V,A
	Vbmin = np.sqrt(dXm**2 + dYm**2)
	Abmax = np.sqrt(d2Xm**2 + d2Ym**2)
	#find scaling 
	fV = V_min/Vbmin
	fA = fV*fV
	#scale acc, vel, time
	Alon = fA*Ablon
	Alat = fA*Ablat
	sec_time = var/fV # fV*(0..1)
	V = fV*Vb
	dist = V*res/fV # distance between points along the curve
	u = int(t_max/res)
	# clip the velocity and acceleration. use clipped velocity to find clipped acceleration
	for i in range(3):
		cap_V = np.clip(V,0,V_lim)
		# print(cap_V) # you can use this to check what the velocity would look like along the curve
		cap_Alon = np.clip(np.diff(cap_V/sec_time),-lon_Acc_limit,lon_Acc_limit)
		# time instant for min vel:
		
		cap_Alon[:u] *= -1
		V[:-1] = V_min + np.cumsum(cap_Alon)*fV*res
		V[-1] = V[-2] # because terms are lost on diffing

	new_time = np.sum(dist/V)/np.sum(dist/V_min)
	return new_time


def cmp(a,b):
	return (a > b) ^ (a < b)

def get_Curvature(X1,Y1,X2,Y2,X3,Y3,X4,Y4,t):
	Px = np.array([X1,X2,X3,X4])
	Py = np.array([Y1,Y2,Y3,Y4])
	KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
	KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
	KX2 = 6*X1 - 12*X2 + 6*X3
	KY2 = 6*Y1 - 12*Y2 + 6*Y3
	KX3 = 3*(X2 - X1)
	KY3 = 3*(Y2 - Y1)
 	#using newton rhapshody method to find best estimates for curvature
	for i in range(2):
		count = 0
		h =1
		for j in range(3):
			dk,d2K = Curv(t[i],KX1,KX2,KX3,KY1,KY2,KY3)
			if(j>=1):
				last_h = h
				h = dk/d2K
				if(h*last_h<0):
					h = cmp(h,0)*min(m.fabs(last_h/2),m.fabs(h))
			else:
				h = dk/d2K
			if(np.isnan(h)):
				t[i] = 0.5
				break
			t[i] = t[i]-h
			t[i] = check_range(t[i],i)
	Curvature = np.zeros(4)
	t = np.array([t[0],t[1],0,1])
	Curvature = C_from_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3)
	section_time = time_from_P_K_t(t,KX1,KX2,KX3,KY1,KY2,KY3,Px,Py,Curvature)
	var = np.arange(0,1,0.01)
	dk,d2K = Curv(var,KX1,KX2,KX3,KY1,KY2,KY3)
	return Curvature,dk,d2K,section_time

def s_k(X, Y, slope1, destX, destY, slope2):
	int1,int2 = get_Intermediate_Points( slope1, slope2, X, destX, Y, destY)
	t = get_T(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY)
	Curvature,dk,d2K,section_time = get_Curvature(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, t)
	k_st = Curvature[2]
	k_en = Curvature[3]
	Curvature = np.max(np.fabs(Curvature))
	dk = np.max(np.fabs(dk))
	d2K = np.max(np.fabs(d2K))
	s = arc_length(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY)
	return Curvature,dk,d2K,s,k_st,k_en,section_time

def net_cost(X,Y,slope):
	n = len(X)
	cost = np.zeros_like(slope)
	for i in range(1,n):
		s = (i-1)%n
		f = i%n
		X1 = X[f]
		Y1 = Y[f]
		k1, dk1, d2k1, s1, k1_st, k1_en,section_time1 = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])
		s = i%n
		f = (i+1)%n
		X2 = X[s]
		Y2 = Y[s]
		if(distancecalcy(Y[f],Y[s],X[f],X[s])<0.01):
			f+=1
		k2, dk2, d2k2, s2, k2_st, k2_en,section_time2 = s_k(X[s], Y[s], slope[s], X[f], Y[f], slope[f])

		C0_continuity = distancecalcy(Y1,Y2,X1,X2) # only C0 and C2 continuity is checked as C1 continuity already exists. also, C0 continuity is there by default but not in circuits
		C2_continuity = m.fabs(k1_en-k2_st)
		cost[i] += W[0]*Q[0]*k2**2 + W[1]*Q[1]*dk2**2 + W[2]*Q[2]*d2k2**2 + W[3]*Q[3]*s2**2 + W[4]*Q[4]*C0_continuity**2 + W[5]*Q[5]*C2_continuity + Q[6]*section_time2
	return cost


def get_bezier_track(X,Y,slope):
	bx = np.zeros(len(X)*20)
	by = np.zeros(len(X)*20)
	for i in range(len(X)-1):
		k = i*20
		bx[k:k+20],by[k:k+20] = get_bezier(X[i],X[i+1],Y[i],Y[i+1],slope[i],slope[i+1])
	return bx,by
