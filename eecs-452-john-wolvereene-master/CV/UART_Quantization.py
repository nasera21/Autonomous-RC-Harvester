# -*- coding: utf-8 -*-
"""
Created on Sat Mar 27 02:06:10 2021

@author: Katie
"""

import numpy as np

#1.1
totBit = 8
def fpq(x,s):
    #(8, 2^s*x)
    xq = np.round(x*(2.**s)) #mantissa
    #saturation (I think this works? have to saturate at 31.875)
    xq[x>31.875] = np.round(31.875*(2.**s))
    xq = xq.astype(np.uint8)  #will need to change to uint16 if we want 16 bit
    b = totBit
    return (xq, (b,s))

def back2float(x):
    XQ = x[0]
    S = x[1][1]
    flt = XQ/2.**S
    return (flt)

#can't exceed 8 bits - unsigned integer bc assume always pos
#dist < 32  
#(8, A)
#dist needs 5 bits left of decimal
#this leaves 3 bits for the mantissa (1/8 inch resolution)
#(8, 3)

#testVals = np.array([20.57138903098, 15, 1.81035, 10.895781, 582.1, 31.99])
#quantDist = fpq(testVals, 3)
#floatDist = back2float(quantDist)






    