2# -*- coding: utf-8 -*-
"""
Created on Wed Mar 10 18:59:27 2021

@author: Katie
"""
import numpy as np
from matplotlib import pyplot as plt


#some issues with variables being redefined but the methods themselves should be good

#THINGS YOU NEED TO FIND DURING CALIBRATION
pix = 2*79; # number of pixels representing the width of the object at calibration distance
D = 4; #calibration distance
w_o = 1.5; #width of object in real life (same units as D)
F = pix*D/w_o;

widths_px = np.array([100, 90, 80, 70, 60, 50, 101, 93, 82, 71, 62, 52, 
                      98, 89, 77, 65, 59, 48, 100, 90, 80, 70, 60, 50])
distCtr_px = np.array([300, 270, 230, 200, 170, 150, 300, 270, 230, 200, 170, 150,
                       300, 270, 230, 200, 170, 150, 310, 275, 241, 209, 178, 155])
distCtr_px = distCtr_px*3

D_forward = w_o*F/widths_px
D_fromctr = (distCtr_px/widths_px)*w_o

x = D_fromctr
y = D_forward

from sklearn.linear_model import LinearRegression

# transforming the data to include another axis
x = x[:, np.newaxis]
y = y[:, np.newaxis]

#flip axis to make function
xmod = y
ymod = x

model = LinearRegression()
model.fit(xmod, ymod)
y_pred = model.predict(xmod)

#move the polyfit so it's in BEHIND the values
diffsMax = np.max(y_pred-ymod)
y_pred_Shift = y_pred-diffsMax #switch sides


#get where you are relative to the car
horizDist = model.predict([[0]])
horizDistShift = horizDist - diffsMax

plt.figure(1)
plt.scatter(ymod, xmod, s=10)
plt.plot(y_pred_Shift, xmod, color='r')
plt.xlim([0,20])
plt.scatter(horizDistShift, 0)
plt.show()

print(horizDistShift) #THIS IS THE DISTANCE FROM WALL