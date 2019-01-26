import matplotlib.pyplot as plt
import numpy as np
import math as meth

data = np.load('LUCIFER_log_2019_1_27_0_13.npy')

print(data.shape)
lat = data[int(len(data)/2):]
lon = data[:int(len(data)/2)]
print(lat[0],lon[0])

plt.title('')
plt.plot(lon, lat)


plt.show()