###############
### INCLUDE ###
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import numpy as np

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
h_val = 0  # H-value of ball for difference image
thold_val =  130 # Threshold value for difference image
v_val = 70  # V-value of ball for difference image
r_val = 200 # R-val of stake for diff image
b_val = 20 #B-val max of stake for diff image
# Empty callback function for slider updates
def nothing(x):
    pass
#Make trackbar window
#img = np.zeros((300,300,3), np.uint8)
cv2.namedWindow('image')
# Create trackbars for thold and image
cv2.createTrackbar('thold','image',thold_val,255,nothing)
cv2.createTrackbar('h','image',h_val,255,nothing)
cv2.createTrackbar('v','image',v_val,100,nothing)
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
    cv2.imshow('diffIMh', diffImh)
    # Box filter
    postBfilterh = cv2.boxFilter(diffImh, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    trash, hthresh = cv2.threshold(postBfilterh,thold_val,255,cv2.THRESH_BINARY_INV)
    
    #v
    diffImv = color_subtractV(image, v_val)
    cv2.imshow('diffIMv', diffImv)
    # Box filter
    postBfilterv = cv2.boxFilter(diffImv, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    vthold_val = 50
    trash, vthresh = cv2.threshold(postBfilterv,vthold_val,100,cv2.THRESH_BINARY_INV)
    
    #r
    diffImr = color_subtractR(image, r_val)
    cv2.imshow('diffIMr', diffImr)
    # Box filter
    postBfilterR = cv2.boxFilter(diffImr, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    hthold_val = 50
    trash, rthresh = cv2.threshold(postBfilterR,hthold_val,100,cv2.THRESH_BINARY_INV)
    
    #b
    diffImb = color_subtractB(image, b_val)
    cv2.imshow('diffIMb', diffImb)
    # Box filter
    postBfilterb = cv2.boxFilter(diffImb, -1, (5,5))
    #print(np.shape(postBfilter))
    # Threshold
    bthold_val = 50
    trash, bthresh = cv2.threshold(postBfilterb,bthold_val,100,cv2.THRESH_BINARY_INV)
    
    
    threshTot = hthresh*vthresh*rthresh*bthresh
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(threshTot,kernel,iterations = 1)
    cv2.imshow('IMG', erosion)
    
    ##########
    #CONTOURS#
    ##########    
    # Use OpenCV's findContours function
    contours, hierarchy = cv2.findContours(erosion,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contour = 0
    widthArr = []
    middleArr = []
    hImg, wImg = erosion.shape
    minArea = 25*25
    for contour in contours:
        (x,y,w,h) = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > (w*h*0.60) and area > minArea:
            cv2.rectangle(image,(x,y),(x+w,y+h),(200,0,0),2)
            widthArr.append(w)
            middleArr.append((w/2)+((wImg/2)-((x+w))))
    #print(widthArr)
    #print(middleArr)
    
    ###################
    #Focal Length Detection
    ###################
    #1.5 inch ball diameter
    #F = PD/W
    #P - pixels, D = distance, W - actual width
    pix = 2*79 #with front of ball at 4 inches
    D = 4 #4 in away
    w_o = 1.5 #1.5 in diameter
    F = pix*D/w_o #F = 210.6666
    widthArr = np.array([widthArr]); #make np arrays
    middleArr = np.array([middleArr]);
    widthArr[widthArr == 0] = 10; #avoid nans
    middleArr[widthArr == 0] = 0; #set nans to zero
    D_forward = w_o*F/widthArr;
    D_fromctr = (middleArr/widthArr)*w_o;
    print(D_forward)
    print(D_fromctr)
    mid = 20
    x = mid - D_fromctr
    y = D_forward
    # transforming the data to include another axis
    x = x[:, np.newaxis]
    y = y[:, np.newaxis]
    #flip axis to make function
    xmod = y
    ymod = x
    
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
        plt.imshow(imageFix)
        plt.show()
    
# Close all windows
cv2.destroyAllWindows()

    
