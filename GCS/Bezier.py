import numpy as np
import math as m

RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3

track_width = 4
max_curvature = 0.5

def set_track_width(tw):
	global track_width
	track_width = (tw*1.414)*2.5

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
		# if((i-1)%3==0 and i>=1):
		# 	slope[i]=angle1
		# elif((i-3)%3==0 and i>=3):
		# 	slope[i]=angle2
		# else:
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

def get_Intermediate_Points(slope1, slope2, X1, X2, Y1, Y2):
	global track_width
	int1 = np.zeros(2)
	int2 = np.zeros(2)
	d = distancecalcy(Y2,Y1,X2,X1)
	if(d>track_width):
		d = track_width
	int1[0] = X1 + 0.35*m.cos(slope1*DEG2RAD)*d
	int1[1] = Y1 + 0.35*m.sin(slope1*DEG2RAD)*d
	int2[0] = X2 - 0.35*m.cos(slope2*DEG2RAD)*d
	int2[1] = Y2 - 0.35*m.sin(slope2*DEG2RAD)*d
	return int1,int2

def get_bezier(X1,X2,Y1,Y2,slope1,slope2):
	int1,int2 = get_Intermediate_Points(slope1,slope2,X1,X2,Y1,Y2)
	Px = np.array([X1,int1[0],int2[0],X2])
	Py = np.array([Y1,int1[1],int2[1],Y2])
	t = np.arange(0,1,0.1)
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
	denominator = m.sqrt(denominator)
	del3Y = 2*KY1
	del3X = 2*KX1
	second_denominator = denominator*dummy 
	dK = ((del3Y*delX - del3X*delY)/denominator) - (3*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator)
	sub_term_1 = (delX*del2Y - del2X*delY)
	sub_term_2 = 2*(delX*del2X + delY*del2Y)
	third_denominator = m.fabs(second_denominator*dummy)
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
	denominator = m.sqrt(denominator)
	Curvature = ((delX*del2Y) - (delY*del2X))
	Curvature /= denominator
	return Curvature

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
			t[i] = t[i]-h
			t[i] = check_range(t[i],i)
	Curvature = np.zeros(2)
	Curvature[0] = C_from_K_t(t[0],KX1,KX2,KX3,KY1,KY2,KY3)
	Curvature[1] = C_from_K_t(t[1],KX1,KX2,KX3,KY1,KY2,KY3)
	return Curvature

def s_k(X, Y, slope1, destX, destY, slope2):
	int1,int2 = get_Intermediate_Points( slope1, slope2, X, destX, Y, destY)
	t = get_T(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY)
	Curvature = np.max(np.fabs(get_Curvature(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, t)))
	Curvature *= (Curvature/max_curvature)**2
	s = arc_length(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY)
	return Curvature

def get_bezier_track(X,Y,slope):
	bx = np.zeros(len(X)*10)
	by = np.zeros(len(X)*10)
	for i in range(len(X)-1):
		k = i*10
		bx[k:k+10],by[k:k+10] = get_bezier(X[i],X[i+1],Y[i],Y[i+1],slope[i],slope[i+1])
	return bx,by