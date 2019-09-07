import numpy as np 
import matplotlib.pyplot as plt
import math as m

RAD2DEG = 57.3
DEG2METER = 111392.84
DEG2RAD = 1/57.3

def distancecalcy(y1,y2,x1,x2,i):
  delX = (x2-x1);
  delY = (y2-y1);
  delX *= delX;
  delY *= delY;
  if(i==1):
    delX *= DEG2METER;
    delY *= DEG2METER;
    return  m.sqrt(delX + delY);   
  else:
    return m.sqrt(delX + delY);   

def anglecalcy(x1,x2,y1,y2):
  angle = RAD2DEG*m.atan2((y2-y1),(x2-x1));
  if(angle<0):
    angle += 360;
  return angle;

# data = np.load('LUCIFER_WP.npy')
# data_X = data[:,0]
# data_Y = data[:,1]

X = np.array([0, 3, 6, 3, 1,-1,  1, 2, 4, 2, 1,-0.5,-1.4,-2,-2.7,-4,-4.5,-6.5,-3,0])
Y = np.array([0, 0, 3, 6, 6,4.5, 3, 3, 2, 1, 1, 1.4, 2.5, 4, 5.4, 6, 6.0, 3.0, 0,0])
theta = 0/57.3
rot_X = m.cos(theta)*X - m.sin(theta)*Y
rot_Y = m.sin(theta)*X + m.cos(theta)*Y

X = rot_X
Y = rot_Y
slope = np.empty_like(X) 

circuit = False
if(distancecalcy(Y[0],Y[-1],X[0],X[-1],0)<1):
	circuit = True

for i in range(1,len(X)-1):
	angle1 = anglecalcy( X[i-1], X[i], Y[i-1], Y[i] )
	angle2 = anglecalcy( X[i], X[i+1], Y[i], Y[i+1] )
	if(m.fabs(angle1 - angle2) > 180):
		angle1 -= 360
	if((i-1)%3==0 and i>=1):
		slope[i]=angle1
	elif((i-3)%3==0 and i>=3):
		slope[i]=angle2
	else:
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
	slope[-1] = anglecalcy( X[-2], X[-1], Y[-2], Y[-1] );

def get_bezier(X1,X2,Y1,Y2,slope1,slope2):
	d = distancecalcy(Y2,Y1,X2,X1,0)
	int1 = np.zeros(2)
	int2 = np.zeros(2)
	if(d>10):
		d = 10;
	int1[0] = X1 + 0.35*m.cos(slope1*DEG2RAD)*d; 
	int1[1] = Y1 + 0.35*m.sin(slope1*DEG2RAD)*d;
	int2[0] = X2 - 0.35*m.cos(slope2*DEG2RAD)*d; 
	int2[1] = Y2 - 0.35*m.sin(slope2*DEG2RAD)*d;

	Px = np.array([X1,int1[0],int2[0],X2])
	Py = np.array([Y1,int1[1],int2[1],Y2])
	t = np.arange(0,1,0.1)
	T = np.array([(1-t)**3,3*t*(1-t)**2,3*t*t*(1-t),t**3])
	Bx = T[0]*Px[0] + T[1]*Px[1] + T[2]*Px[2] + T[3]*Px[3]
	By = T[0]*Py[0] + T[1]*Py[1] + T[2]*Py[2] + T[3]*Py[3]
	return Bx,By,int1,int2

bx = np.zeros(len(X)*10)
by = np.zeros(len(X)*10)
int1 = np.zeros((len(X),2))
int2 = np.zeros((len(X),2))
for i in range(len(X)-1):
	k = i*10
	bx[k:k+10],by[k:k+10],int1[i],int2[i] = get_bezier(X[i],X[i+1],Y[i],Y[i+1],slope[i],slope[i+1])

print(slope)
plt.axis('equal')
plt.plot(rot_X,rot_Y)
plt.scatter(bx,by)
# plt.scatter(int1[:,0],int1[:,1],label='int1')
# plt.scatter(int2[:,0],int2[:,1],label='int2')
plt.legend()

# wp = []
# for i in range(len(X)):
# 	wp.append([X[i],Y[i]])
# wp = np.array(wp)
# np.save('LUCIFER_WP_4.npy',wp)
# data = np.load('LUCIFER_WP_4.npy')
# print(data)


plt.show()
