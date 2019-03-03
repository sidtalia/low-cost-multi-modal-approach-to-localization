import matplotlib.pyplot as plt
import numpy as np
import math as meth

data = np.load('LUCIFER_log_filt_2019_2_28_21_54.npy')
data1 = np.load('LUCIFER_log_gps_2019_2_28_21_54.npy')


print(data.shape)

lat = data[int(len(data)/2):]
# lat = lat[:-40]
lon = data[:int(len(data)/2)]
# lon = lon[:-40]
# speed = np.sqrt(np.gradient(lat)**2 + np.gradient(lon)**2)*111392*10

gps_lat = data1[int(len(data)/2):]
gps_lat -= (gps_lat[0]-lat[0])
# gps_lat = gps_lat[:-40]
gps_lon = data1[:int(len(data)/2)]
gps_lon -= (gps_lon[0]-lon[0])
# gps_lon = gps_lon[:-40]
print(gps_lat[0],gps_lon[0]) 

theta = meth.atan2((lat[10] - lat[0]),(lon[10] - lon[0])) - meth.atan2((gps_lat[10]-gps_lat[0]),(gps_lon[10] - gps_lon[0]))
print(theta)
# angle = 57.3*meth.atan((gps_lat[30] - lat[30])/(gps_lon[30] - lon[30]))
# print(angle)

plt.title('')
plt.plot(lat, lon, label='filtered')
plt.plot(gps_lat,gps_lon, label='raw gps')
# plt.plot(np.arange(0,len(lat)/10,0.1),speed,label='speed')
plt.legend()

plt.show()

# def sinu(x):
#     B = 4/meth.pi
#     C = -B/meth.pi

#     y = B*x + C*x*np.fabs(x);

#     P = 0.225;
#     y = P*(y*abs(y) - y) + y #   // Q * y + P * y * abs(y)
#     return y;

# def asinu(x):
# 	return (0.87266462599716477 + 0.69813170079773212*x*x )*x

# a = np.arange(0,1.0,0.001)

# last_MSE = 100
# for i in range(0,15):
# 	x = 0.1*i
# 	C1 = meth.pi/2 - x
# 	C2 = x
# 	y = a*(C1+ C2*a*a*a*a*a*a)
# 	MSE = np.linalg.norm(y-np.arcsin(a))
# 	if(MSE < last_MSE):
# 		save = np.array([C1,C2])
# 		last_MSE = MSE

# z = a*(1+0.57*a*a*a*a*a*a)
# MSE1 = np.linalg.norm(z-np.arcsin(a))

# # print(save,MSE/15)
# # print(y[0],y[-1])
# # y = x*x*x*x/(0.01+x*x*x*x)
# y = a*(save[0] +save[1]*a*a*a*a*a*a) 
# MSE2 = np.linalg.norm(y-np.arcsin(a))
# print(MSE1,MSE2)
# # plt.title('')
# # # plt.ylim((-1,1))
# plt.plot(a, y, label='internet')
# plt.plot(a, z,label='mine')
# plt.plot(a, np.arcsin(a), label ='true')
# plt.legend()
# plt.show()

