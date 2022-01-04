# -*- coding: utf-8 -*-
"""
Created on Wed Mar 31 20:22:26 2021

@author: Katie
"""

import math

#define curves
###########################
#FOCAL DIST CALIBRATION
pix = 2*79; # number of pixels representing the width of the object at calibration distance
D = 4; #calibration distance
w_o = 1; #width of object in real life (same units as D)
F = pix*D/w_o;
###########################

#Define Curves, possible width from angle and distHoriz
wTube = 1.625

#what is the possible triangle that could get it
#use angle and horiz dist to calculate possible pixel widths

def pixMinMax(theta, dh):
    #calibrated that the blindspot angle is 63.5 -> (90-63.5)*2 = 53 degrees across periphery
    #use angle of car to wall theta to get the two bounding angles phi and phi'
    phi = theta+63.5 #63.5 IS A CALIBRATED VALUE
    phiPrime = theta + 63.5 + 53
    #use phi and phiPrime along with horiz dist
    # to calculate the range of possible forward distances
    if phi>=90:
        dfMin = math.inf
    else:
        dfMin = dh/math.cos(phi*(math.pi/180))
    if phiPrime>=90:
        dfMax = math.inf
    else:
        dfMax = dh/math.cos(phiPrime*(math.pi/180))
    #use focal dist to calculate pixels, minDist is maxPix
    pixMax = F*wTube/dfMin #F IS A CALIBRATED VALUE
    pixMin = F*wTube/dfMax
    return pixMax, pixMin

#Define curves, possible %screen from angle and distHoriz
###FUTURE WORK?
#Use distance sensor data?

#have a set of pixWidths
pixTestSet = [40, 30, 20, 10]
#need to find what set of possible ranges they could be in
#how would you ransac this?
#can we use more data abt pos of obj in frame?

currentTheta = -40
currentDh = 10
pixMax, pixMin = pixMinMax(currentTheta, currentDh)

if(pixMax > max(pixTestSet)) or (pixMin < min(pixTestSet)):
    pixMax1, pixMin1 = pixMinMax(currentTheta+1, currentDh)
    pixMax2, pixMin2 = pixMinMax(currentTheta, currentDh+1)
    if((max(pixTestSet)-pixMax1)>(max(pixTestSet)-pixMax)) or ((pixMin1-min(pixTestSet))>(pixMin-min(pixTestSet))):
        newTheta = currentTheta+1
    if((max(pixTestSet)-pixMax2)>(max(pixTestSet)-pixMax)) or ((pixMin2-min(pixTestSet))>(pixMin-min(pixTestSet))):
        newDh = currentDh+1
        
#if distance sensor data is within these resonable values, use that as currentDh

