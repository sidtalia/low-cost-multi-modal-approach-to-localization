import cv2
import numpy as np
import math as m

image = cv2.imread("coneslam.png")
Xmax = image.shape[1]//2
Ymax = image.shape[0]//2
X_Center = image.shape[1]//2
Y_Center = image.shape[0]//2

fov_h = 60/57.3
K_h = m.tan(fov_h/2)
fov_v = fov_h*120/160
K_v = m.tan(fov_v/2)
H = 0.11
xreal = 0.5 + 3.02*m.tan(1.5/57.3)
yreal = 2.02

ypix = int(Y_Center + H*Ymax/(yreal*K_v))
xpix = int(X_Center + xreal*Xmax/(yreal*K_h))
#239 135
print()

center_coordinates = (xpix,ypix)
radius = 5
thickness = -1
color = (255,0,0)
image = cv2.circle(image, center_coordinates, radius, color, thickness)
cv2.imshow('img',image)
cv2.waitKey(0)
cv2.destroyAllWindows()