import numpy as np 
import matplotlib.pyplot as plt
import math as m
import time

now = time.time()
a = 50/57.3
t = np.arange(0,1,0.01)
Px = np.array([0,0.4*m.cos(a),1-0.4*m.cos(a),1])
Py = np.array([0,0.4*m.sin(a),0.4*m.sin(a),0])

# plt.xlim(0,15)
# plt.ylim(0,15)

def get_Intermediate_Points(slope1, slope2, X1, X2, Y1, Y2):
	int1 = np.zeros(2)
	int2 = np.zeros(2)
	d = distancecalcy(Y2,Y1,X2,X1)
	if(d>20):
		d = 20
	int1[0] = X1 - 0.4*m.cos(slope1)*d
	int1[1] = Y1 - 0.4*m.sin(slope1)*d
	int2[0] = X2 - 0.4*m.cos(slope2)*d
	int2[1] = Y2 - 0.4*m.sin(slope2)*d
	return int1,int2

def sigmoid(x):
	return m.sqrt(1+x**2) -1

dK = np.zeros(1000)
Curvature = np.zeros(1000)
d2k = np.zeros(1000)


def Curv(t,KX1,KX2,KX3,KY1,KY2,KY3):
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
	Curvature = Curvature
	del3Y = 2*KY1
	del3X = 2*KX1
	second_denominator = denominator*dummy 
	dK = ((del3Y*delX - del3X*delY)/denominator) - (3*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator)
	del4X = 0
	del4Y = 0
	sub_term_1 = (delX*del2Y - del2X*delY)
	sub_term_2 = 2*(delX*del2X + delY*del2Y)
	third_denominator = m.fabs(second_denominator*dummy)
	sub_term_3 = (del3Y*delX - del3X*delY)
	sub_term_4 = 2*(del2X**2 + del2Y**2 + del3X*delX+del3Y*delY)
	sub_term_5 = -del4X*delY - del3X*del2Y + del3Y*del2X + del4Y*delX
	term_1 = 3.75*(sub_term_1*(sub_term_2**2))/third_denominator
	term_2 = -3*(sub_term_3*sub_term_2)/second_denominator
	term_3 = -1.5*(sub_term_1*sub_term_4)/second_denominator
	term_4 = sub_term_5/denominator
	d2K = term_1 + term_2 + term_3 + term_4
	return dK,d2K

def calc(X1,X2,X3,X4,Y1,Y2,Y3,Y4):
	Px = np.array([X1,X2,X3,X4])
	Py = np.array([Y1,Y2,Y3,Y4])
	angle1 = m.atan2((Py[1]-Py[0]),(Px[1]-Px[0]))
	angle1_comp = angle1-m.pi
	angle3 = m.atan2((Py[3]-Py[2]),(Px[3]-Px[2]))
	angle3_comp = angle3-m.pi
	angle2 = m.atan2((Py[2]-Py[1]),(Px[2]-Px[1]))
	angle4 = m.atan2((Py[3]-Py[0]),(Px[3]-Px[0]))

	a1 = (angle2 - angle1_comp)
	b1 = (angle1 - angle4)
	t1 = 0.5*(a1/(a1+b1))**4
	
	KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
	KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
	KX2 = 6*X1 - 12*X2 + 6*X3
	KY2 = 6*Y1 - 12*Y2 + 6*Y3
	KX3 = 3*(X2 - X1)
	KY3 = 3*(Y2 - Y1)

	h = 1
	t = t1 
	print(t1)
	count = 0
	while m.fabs(h)>0.01 and count<2:
		dk,d2K = Curv(t,KX1,KX2,KX3,KY1,KY2,KY3)
		if(count>=1):
			last_h = h
			h = dk/d2K
			if(h*last_h<0):
				h = cmp(h,0)*min(m.fabs(last_h/2),m.fabs(h))
		else:
			h = dk/d2K
		print("h:",h)
		print("t:",t)
		print(t-h)
		t = t-h
		print("after:",t)
		if t>0.5:
			t=0.5
		if t<=0 :
			t=0.0
		print("guess:",t)
		count+=1
	print("count=",count)
	return t 
		
def curvature_min(X1,X2,X3,X4,Y1,Y2,Y3,Y4):
	now = time.time()
	KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3
	KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3
	KX2 = 6*X1 - 12*X2 + 6*X3
	KY2 = 6*Y1 - 12*Y2 + 6*Y3
	KX3 = 3*(X2 - X1)
	KY3 = 3*(Y2 - Y1)

	global Curvature
	global dK
	for i in range(1000):
		t = i*0.001
		delX = t*t*KX1 + t*KX2 + KX3
		delY = t*t*KY1 + t*KY2 + KY3
		del2X = 2*t*KX1 + KX2
		del2Y = 2*t*KY1 + KY2
		denominator = delX*delX + delY*delY
		dummy = denominator
		denominator *= denominator*denominator
		denominator = m.sqrt(denominator)
		Curvature[i] = ((delX*del2Y) - (delY*del2X))
		Curvature[i] /= denominator
		Curvature[i] = m.fabs(Curvature[i])
		del3Y = 2*KY1
		del3X = 2*KX1
		second_denominator = m.fabs(denominator*dummy) 
		dK[i] = ((del3Y*delX - del3X*delY)/denominator) - (3*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator)
		del4X = 0
		del4Y = 0
		sub_term_1 = (delX*del2Y - del2X*delY)
		sub_term_2 = 2*(delX*del2X + delY*del2Y)
		third_denominator = m.fabs(second_denominator*dummy)
		sub_term_3 = (del3Y*delX - del3X*delY)
		sub_term_4 = 2*(del2X**2 + del2Y**2 + del3X*delX+del3Y*delY)
		sub_term_5 = -del4X*delY - del3X*del2Y + del3Y*del2X + del4Y*delX
		term_1 = 3.75*(sub_term_1*(sub_term_2**2))/third_denominator
		term_2 = -3*(sub_term_3*sub_term_2)/second_denominator
		term_3 = -1.5*(sub_term_1*sub_term_4)/second_denominator
		term_4 = sub_term_5/denominator
		d2k[i] = term_1 + term_2 + term_3 + term_4
		# if i>1:
		# 	d2k[i] = (dK[i]-dK[i-1])/0.001

	peaks = np.where((Curvature[1:-1] > Curvature[0:-2]) * (Curvature[1:-1] > Curvature[2:]))[0] + 1
	peaks = peaks*0.001
	# print(peaks)
	print("dt=",time.time()-now)
	try:
		a = peaks[1]
		return peaks
	except:
		try:
			a = peaks[0]
			return np.array([peaks[0],peaks[0]])
		except:
			return np.zeros(2)

T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])

Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]

# t = root1
ds1 = ((Px[1]-Px[0])**2 + (Py[1]-Py[0])**2)**0.5
angle1 = m.atan2((Py[1]-Py[0]),(Px[1]-Px[0]))
angle1_comp = angle1-m.pi

ds3 = ((Px[3]-Px[2])**2 + (Py[3]-Py[2])**2)**0.5
angle3 = m.atan2((Py[3]-Py[2]),(Px[3]-Px[2]))

ds2 = ((Px[2]-Px[1])**2 + (Py[2]-Py[1])**2)**0.5
angle2 = m.atan2((Py[2]-Py[1]),(Px[2]-Px[1]))

ds4 = ((Px[3]-Px[0])**2 + (Py[3]-Py[0])**2)**0.5
angle4 = m.atan2((Py[3]-Py[0]),(Px[3]-Px[0]))

ds_ = (ds1**2 + ds2**2 + ds3**2)**0.5
L = ds1+ds2+ds3


# def depression(ds1,ds3,ds4):
# 	a = ds1/ds3
# 	x = max(ds1,ds3)/ds4
# 	c0 = np.array([-0.99569034,  0.33836428, -1.07450952,  1.57629097])
# 	c1 = np.array([ 5.16876495, -3.23549476,  0.92659594,  0.31344109])
# 	v = max(c0[0]*a**3 + c0[1]*a**2 + c0[2]*a + c0[3],0.65)
# 	h = min(c1[0]*a**3 + c1[1]*a**2 + c1[2]*a + c1[3],1.2)
# 	x *= h
# 	return v*(x**2/(1+x**2))

def depression(x,h,v):
	x*=h
	return v*(x**2/(1+x**2))

# t = (ds1/m.cos(m.fabs(angle2)))/L
# t = (ds1/(ds3+ds1))*sigmoid(ds1/ds4)
# t = 0.5*ds1/(ds1+ds2) #this one works for opposite ends
a1 = (angle2 - angle1_comp)
b1 = (angle1 - angle4)
t = calc(Px[0],Px[1],Px[2],Px[3],Py[0],Py[1],Py[2],Py[3])
# t = 0.5*(a1/(a1+b1))**4
plt.scatter(t,0)
# print((angle2-angle1)*57.3)
# t = 0.5*ds1/(ds4)
# t = depression(ds1,ds3,ds4)
print(t)
# t = m.fabs(m.atan(ds1/ds4))/m.fabs(m.atan(ds2/ds3)
T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
max1_Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
max1_By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]

# t = (L-ds3/m.cos(m.fabs(angle2)))/L
# t = 
t = 1-(0.5*ds3/(ds3+ds2)) #this one works for opposite ends
plt.scatter(t,0)
# t = 1 - 0.5*ds3/ds4
# t = 1-depression(ds1,ds3,ds4)
print(t)
T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
max2_Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
max2_By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]

peaks = curvature_min(Px[0],Px[1],Px[2],Px[3],Py[0],Py[1],Py[2],Py[3])
t = peaks[0]
T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
anal1_Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
anal1_By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]

t = peaks[1]
T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
anal2_Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
anal2_By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]

print(time.time()-now)

plt.title('')
plt.plot(Px,Py)
plt.scatter(max1_Bx,max1_By,label = 'maxima1')
plt.scatter(max2_Bx,max2_By,label = 'maxima2')
plt.scatter(anal1_Bx,anal1_By,label = 'analytical 1')
plt.scatter(anal2_Bx,anal2_By,label = 'analytical 2')
plt.plot(Bx, By, label='bezier curve')
plt.plot(np.arange(0,1,1e-3),0.5*dK/max(np.fabs(dK)),label="curvature derivative")
plt.plot(np.arange(0,1,1e-3),0.5*Curvature/max(np.fabs(Curvature)),label="curvature")
plt.plot(np.arange(0,1,1e-3),0.5*d2k/max(np.fabs(d2k)),label="d2K")
plt.plot(np.arange(0,1,1e-2),np.zeros(100))
# plt.legend()
plt.axis("equal")

plt.show()
