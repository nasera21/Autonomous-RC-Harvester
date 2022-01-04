###############
### INCLUDE ###
import math as mt
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import numpy as np
import serial
from serial import Serial
import time
import UART_rpiSerial
import UART_Quantization
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN)

#file1 = open("MyFile.txt","a")
#file1.write("test")
#file1.write("\n")
#file1.close()
#Init Arduino Serial Port 
ser = serial.Serial("/dev/ttyS0", 9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
########################
### HELPER FUNCTIONS ###
########################
# Function for performing color subtraction
# Inputs:
#   img     Image to have a color subtracted. shape: (N, M, C)
#   color   Color to be subtracted.  Integer ranging from 0-255
# Output:
#   Grayscale image of the size as img with higher intensity denoting greater color difference. shape: (N, M)
def color_subtract(img, color):

    # TODO: Convert img to HSV format
    imgFix = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # TODO: Extract only h-values (2D Array)
    hVals = imgFix[:, :, 0]
    # TODO: Take the difference of each pixel and the chosen H-value
    hVals.astype(np.float64)
    hDiff = hVals - color
    # TODO: Take absolute value of the pixels 
    hDiff = np.absolute(hDiff)
    final_img = hDiff
    final_img.astype(np.uint8)
    return final_img

def color_subtractV(img, color):

    # TODO: Convert img to HSV format
    imgFix = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # TODO: Extract only v-values (2D Array)
    vVals = imgFix[:, :, 2]
    # If you're above the threshold set to 0
    vFin = vVals
    vFin[vFin < color] = color-1
    vFin[vFin>=color] = 0
    final_img = vFin
    final_img.astype(np.uint8)
    return final_img


def color_subtractR(img, color):

    # TODO: Convert img to RGB from BGR format
    imgFix = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # TODO: Extract only v-values (2D Array)
    vVals = imgFix[:, :, 0]
    # If you're above the threshold set to 0
    vFin = vVals
    vFin[vFin < color] = color-1
    vFin[vFin>=color] = 0
    final_img = vFin
    final_img.astype(np.uint8)
    return final_img

def color_subtractB(img, color):

    # TODO: Convert img to RGB from BGR format
    imgFix = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # TODO: Extract only v-values (2D Array)
    vVals = imgFix[:, :, 2]
    # If you're above the threshold set to 0
    vFin = vVals
    vFin[vFin < color] = 0
    vFin[vFin>=color] = color+1
    final_img = vFin
    final_img.astype(np.uint8)
    return final_img

######################
### INITIALIZATION ###
######################
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

#################
### TRACKBARS ###
#################
#Default Trackbar Values
h_val = 173  # H-value of ball for difference image
thold_val =  85 # Threshold value for difference image
v_val = 47 # V-value of ball for difference image
r_val = 72 # R-val of stake for diff image
b_val = 60 #B-val max of stake for diff image
# Empty callback function for slider updates
def nothing(x):
    pass
#Make trackbar window
#img = np.zeros((300,300,3), np.uint8)
cv2.namedWindow('image')
# Create trackbars for thold and image
cv2.createTrackbar('thold','image',thold_val,255,nothing)
cv2.createTrackbar('h','image',h_val,255,nothing)
cv2.createTrackbar('v','image',v_val,150,nothing)
cv2.createTrackbar('r','image',r_val,255,nothing)
cv2.createTrackbar('b','image',b_val,255,nothing)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #######################
    ### READ INPUT DATA ###
    #######################
    #Read trackbar positions
    thold_val = cv2.getTrackbarPos('thold','image')
    h_val = cv2.getTrackbarPos('h','image')
    v_val = cv2.getTrackbarPos('v','image')
    r_val = cv2.getTrackbarPos('r','image')
    b_val = cv2.getTrackbarPos('b','image')
    # Grab raw NumPy array representing the image
    image = frame.array
    
    ##########################
    ### COLOR THRESHOLDING ###
    ##########################
    # Use difference image method
    # Calculate difference image
    # Generate difference image using color subtract function (Section 3.1)
    #h
    diffImh = color_subtract(image, h_val)
    #cv2.imshow('diffIMh', diffImh)
    # Box filter
    postBfilterh = cv2.boxFilter(diffImh, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    trash, hthresh = cv2.threshold(postBfilterh,thold_val,255,cv2.THRESH_BINARY_INV)
    cv2.imshow('Hthresh', hthresh)
    #v
    diffImv = color_subtractV(image, v_val)
    # Box filter
    postBfilterv = cv2.boxFilter(diffImv, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    vthold_val = 50
    trash, vthresh = cv2.threshold(postBfilterv,vthold_val,150,cv2.THRESH_BINARY_INV)
    cv2.imshow('Vthresh', vthresh)

    #r
    diffImr = color_subtractR(image, r_val)
    # Box filter
    postBfilterR = cv2.boxFilter(diffImr, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    rthold_val = 10
    trash, rthresh = cv2.threshold(postBfilterR,rthold_val,100,cv2.THRESH_BINARY_INV)
    cv2.imshow('rthresh', rthresh)

    #b
    diffImb = color_subtractB(image, b_val)
    # Box filter
    postBfilterb = cv2.boxFilter(diffImb, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    bthold_val = 10
    trash, bthresh = cv2.threshold(postBfilterb,bthold_val,100,cv2.THRESH_BINARY_INV)
    cv2.imshow('bthresh', bthresh)

    
    threshTot = hthresh*vthresh*rthresh*bthresh
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(threshTot, kernel, iterations = 1)
    cv2.imshow('IMG', erosion)
    
    ##########
    #CONTOURS#
    ##########    
    # Use OpenCV's findContours function
#LIST
    contours, hierarchy = cv2.findContours(erosion,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contour = 0
    widthArrr = []
    middleArrr = []
    xArr = []
    hImg, wImg = erosion.shape
    minArea = 25*25

    for contour in contours:
        (x,y,w,h) = cv2.boundingRect(contour)
        if w*2>h: #wrong shape
            continue
        
        if cv2.contourArea(contour) > 10000: #too large
            continue
        
        if cv2.contourArea(contour) < 100: #too small
            continue
    
        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)

        rect = cv2.minAreaRect(contour)
        (rx, ry), (width, height), angle = rect
        
        #width and height seem to get mixed up sometimes
        #always choose the larger value as the height
        widthHold = width
        heightHold = height
        if widthHold>heightHold:
            width = heightHold
            height = widthHold
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(image, [box],0,(200,0,0),2) 

        widthArrr.append(width)
        xArr.append(x)
        middleArrr.append((w/2)+((wImg/2)-((x+w))))
    #print(widthArr)
    #print(middleArr)
    
    ###################
    #Focal Length Detection
    ###################
    #1.5 inch ball diameter
    #F = PD/W
    #P - pixels, D = distance, W - actual width
    pix = 2*79; # number of pixels representing the width of the object at calibration distance
    D = 4; #calibration distance
    w_o = 1.5; #width of object in real life (same units as D)
    F = pix*D/w_o
    F = F*1.2

    if not widthArrr:
        widthArrr.append(1)
    if not middleArrr:
        middleArrr.append(1)
        
    widthArrOld = np.array(widthArrr)
    middleArr = np.array(middleArrr)
    xArr = np.array(xArr)
    
    #adjustments for planar projection
    beta = mt.atan(F/xArr)
    alpha = mt.atan(F/(xArr+widthArrOld))-beta
    widthArrUpdated = 2*F*(mt.tan(alpha/2))
    widthArr = widthArrUpdated
    
    
    def reject_outliers(data):
        u = np.mean(data)
        s = np.std(data)
        data_filtered = data[(data>(u-2*s)) & (data<(u+2*s))]
        return data_filtered
    
    
    
    #actual dist calcs
    D_forward = w_o*F/widthArr
    D_fromctr = (middleArr/widthArr)*w_o
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
    #TEMPORARILY DISABLING THIS UNTIL WE CAN EXCLUDE OUTLIERS
    y_pred_Shift = y_pred
    
    #get angle of the line (NOT angle of the car)
    ymod0 = model.predict([[0]])
    ymod1 = model.predict([[1]])
    theta = (180/mt.pi)*mt.atan(1/abs(ymod1-ymod0))
    #shift because camera is already on an angle relative to car
    theta = theta ##########
    
    print("theta:")
    print(theta)
    
    #get where you are relative to the row
    #want to be perpendicular to the line
    horizDist = model.predict([[0]])
    horizDistShift =abs(horizDist - diffsMax)
    distFinal = mt.sin((mt.pi/180)*theta)*horizDistShift
    print("distance:")
    print(distFinal[0][0])
    
    #print(horizDistShift[0][0])
    #UART_rpiSerial.send_message(int(horizDistShift))
    #UART_rpiSerial.send_message(int(distFinal))
    
    horizDist = model.predict([[0]])
    horizDistShift = abs(horizDist - diffsMax)
    print(horizDistShift[0][0])
    
    
    #print(x)
    
    if GPIO.input(17):
        #arduinoMsg1, arduinoMsg2 = UART_rpiSerial.check_format()
        UART_rpiSerial.send_message(horizDistShift[0][0])
        time.sleep(0.3)
        if ser.inWaiting() > 0:
            incomingData = ser.read(1)
            print(incomingData)
        
    ####################################################
    ### DISPLAY THE IMAGE, DEAL WITH KEYPRESSES, ETC ###
    ####################################################
    # Show the image
    cv2.imshow("Raw Image", image)
    # Get keypress
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    # if the 'c' key is pressed, capture and display the camera image
    ######PROBABLY WANT TO PUT THE STAKE PLOT IN HERE
    if key == ord("c"):
        imageFix = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plt.figure(1)
        plt.imshow(imageFix)
        plt.figure(2)
        plt.scatter(ymod, xmod)
        plt.plot(y_pred_Shift, xmod)
        #plt.xlim([0,20])
        #plt.scatter(horizDistShift, 0)
        plt.show()

# Close all windows
cv2.destroyAllWindows()

    
