# -*- coding: utf-8 -*-
"""
Created on Sat Mar 27 02:06:10 2021

@author: Katie
"""

import numpy as np

#fpq quantizes a value x into a certain number of bits with mantissa s
#input: x, the value to be quantized and s, the bits of mantissa
#output: tuple of the form (xq, (b, s)) where xq is the quantized value, b is the total number of bits, s is the bits in mantissa
#requires that totBit (total number of bits) be a predefined constant
totBit = 8
def fpq(x,s):
    #(8, 2^s*x)
    xq = np.round(x*(2.**s)) #mantissa
    #saturation, saturate at 31.875
    xq[x>31.875] = np.round(31.875*(2.**s))
    xq = xq.astype(np.uint8)  #making a uint8
    b = totBit
    return (xq, (b,s))


#back2float takes a quantized value and returns it's floating point value
#input: x, the quantized value in tuple form (xq, (b, s)) where xq is the quantized value, b is the total number of bits, s is the bits in mantissa
#output: floating point number
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









    
