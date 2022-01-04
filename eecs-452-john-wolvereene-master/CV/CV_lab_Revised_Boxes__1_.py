# Skeleton code to be filled in by students for EECS 452 ball tracking lab
# Authors:
#   Ben Simpson
#   Siddharth Venkatesan
#   Ashish Nichanametla

###############
### INCLUDE ###
###############
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
from matplotlib import pyplot as plt
import numpy as np
from timer import Timer

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
    # HINT: Pay attention to integer overflow here. What happens when
    # subtracting unsigned ints and the result is negative?
    # Use integer casting before you subtract and take the absolute value, then
    # use it again when you return a 2D array of unsigned ints as your difference image
    hVals.astype(np.float64)
    hDiff = hVals - color
    # TODO: Take absolute value of the pixels 
    hDiff = np.absolute(hDiff)
    final_img = hDiff
    final_img.astype(np.uint8)
    return final_img

# Function to find the centroid and radius of a detected region
# Inputs:
#   img         Binary image
#   USE_IDX_IMG Binary flag. If true, use the index image in calculations; if
#               false, use a double nested for loop.
# Outputs:
#   center      2-tuple denoting centroid
#   radius      Radius of found circle
def identify_ball(img, USE_IDX_IMG):
    # Find centroid and number of pixels considered valid
    h, w = img.shape

    # Double-nested for loop code
    if USE_IDX_IMG == False:
        k = 0
        x_sum = 0
        y_sum = 0
        for y in range(w):
            for x in range(h):
                if img[x,y] != 0:    #Uncomment this line when editing here
                    # TODO: Calculate x_sum, y_sum, and k
                    x_sum = x_sum + x
                    y_sum = y_sum + y
                    k = k + 1
                    pass


        if(k != 0):
            # TODO: Calculate the center and radius using x_sum, y_sum, and k.
            cx = x_sum/k
            cy = y_sum/k
            rad = int(sqrt(k/pi))
            pass
        else:
            # TODO: Don't forget to account for the boundary condition where k = 0.
            cx = 0
            cy = 0
            rad = 0
            pass
        center = (cx,cy)
        radius = rad

    # Use index image
    else:
        # Calculate number of orange pixels
        k = np.sum(img)

        if k == 0:
            # No orange pixels.  Return some default value
            return (0,0), 0

        # Index image vectors
        x_idx = np.expand_dims(np.arange(w),0)
        y_idx = np.expand_dims(np.arange(h),1)

        # TODO: Calculate the center and radius using the index image vectors
        #       and numpy commands
        cx = np.sum(np.dot(img, np.transpose(x_idx))) // k
        cy = np.sum(np.dot(np.transpose(y_idx), img)) // k
        center = (cx,cy)
        radius = int(np.sqrt(k/np.pi//255))
    return center, radius

# Function to find the centroid and radius of a detected region using OpenCV's
# built-in functions.
# Inputs:
#   img         Binary image
# Outputs:
#   center      2-tuple denoting centroid
#   radius      Radius of found circle
def contours_localization(img):

    # TODO: Use OpenCV's findContours function to identify contours in img.
    #       Assume the biggest contour is the ball and determine its center and
    #       radius. Do not forget to deal with the boundary case where no
    #       contours are found.

    center, radius = (0,0), 0

    return center, radius

################################
### CONSTANTS AND PARAMETERS ###
################################
# Flag for indicating how we perform color thresholding
USE_DIFFERENCE_IMAGE = True
# Flag for indicating how we perform ball localization
USE_LOCALIZATION_HEURISTIC = False
# Flag to indicate computation method for localization heuristic
USE_IDX_IMG = True

# Flag for indicating if we want the function timers to print every frame
FTP = True

# Values for color thresholding (default trackbar values)
# TODO: Find and tune these. Uncomment lines below and assign values to variables.
h_val = 9  # H-value of ball for difference image
thold_val =  5 # Threshold value for difference image

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

# Empty callback function for slider updates
def nothing(x):
    pass

# TODO: Make trackbar window
#img = np.zeros((300,300,3), np.uint8)
cv2.namedWindow('image')
# TODO: Create trackbars. Use variables you defined in the CONSTANTS AND
#       PARAMETERS section to initialize the trackbars. (Section 3.4)
cv2.createTrackbar('thold','image',thold_val,255,nothing)
cv2.createTrackbar('h','image',h_val,255,nothing)

##############
### TIMERS ###
##############

color_threshold_timer = Timer(desc="  color threshold",printflag=FTP)
contours_timer = Timer(desc="  contours",printflag=FTP)
img_disp_timer = Timer(desc="  display images",printflag=FTP)
# TODO: Add timers for difference image calculation, box filtering, and thresholding (Section 4)
imDiff_timer = Timer(desc="  difference image",printflag=FTP)
box_filter_timer = Timer(desc="  box filter",printflag=FTP)
threshold_image_timer = Timer(desc="  threshold image",printflag=FTP)


#################
### MAIN LOOP ###
#################

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame_start = time.time() # Start timer for whole frame

    #######################
    ### READ INPUT DATA ###
    #######################
    # TODO: Read trackbar positions (Section 3.4)
    thold_val = cv2.getTrackbarPos('thold','image')
    h_val = cv2.getTrackbarPos('h','image')
    # Grab raw NumPy array representing the image
    image = frame.array

    ##########################
    ### COLOR THRESHOLDING ###
    ##########################
    color_threshold_timer.start_time()
    # Use difference image method
    if USE_DIFFERENCE_IMAGE == True:
        thresh = np.zeros_like(image[:, :, 1]) # Delete this line once you add your code (Section 3.2)
        # Calculate difference image
        # TODO: 1) Generate difference image using color subtract function (Section 3.1)
        imDiff_timer.start_time()
        diffIm = color_subtract(image, h_val)
        imDiff_timer.end_time()
        cv2.imshow('diffIM', diffIm)
        
        # TODO: 2) Time function (Section 4)

        # Box filter
        # TODO: 1) Implement box filter (Section 3.3)
        box_filter_timer.start_time()
        postBfilter = cv2.boxFilter(diffIm, -1, (5,5))
        box_filter_timer.end_time()
        #print(np.shape(postBfilter))

        # TODO: 2) Time function (Section 4)

        # Threshold
        # TODO: 1) Threshold (Section 3.2)
        threshold_image_timer.start_time()
        trash, thresh = cv2.threshold(postBfilter,thold_val,255,cv2.THRESH_BINARY_INV)
        threshold_image_timer.end_time()
        cv2.imshow('IMG', thresh)
        # TODO: 2) Time function (Section 4)

    # Use HSV range method
    #else:
        #thresh = np.zeros_like(image[:, :, 1]) # Delete this line once you add your code
        # TODO: Perform HSV thresholding
        # thresh = Your code here

    color_threshold_timer.end_time()

    #########################
    ### BALL LOCALIZATION ###
    #########################
    contours_timer.start_time()
    # Use the heuristic
    if USE_LOCALIZATION_HEURISTIC == True:
        center, radius = identify_ball(thresh, USE_IDX_IMG)

    # Use OpenCV's findContours function
    else:
########################################################################################################################################################
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contour = 0
        widthArr = []
        middleArr = []
        hImg, wImg = thresh.shape
        for contour in contours:
            (x,y,w,h) = cv2.boundingRect(contour)
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
    
###################################################################################################################################################################
        #    center, radius = contours_localization(thresh)
       # print(center)
       # print(radius)

    # Draw the circle
   # cv2.circle(image,center, radius,(0,255,0),2)

    contours_timer.end_time()

    ####################################################
    ### DISPLAY THE IMAGE, DEAL WITH KEYPRESSES, ETC ###
    ####################################################
    # Show the image
    img_disp_timer.start_time()
    cv2.imshow("Raw Image", image)
    # TODO: Show the difference frame (Section 3.1)
    # Hint: Show the "thresh" image in a new window called "Difference Image"
    # TODO: Show the binary frame (Section 3.2)

    img_disp_timer.end_time()

    # Get keypress
    key = cv2.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    # TODO: if the 'c' key is pressed, capture and display the camera image
    # using pyplot (Section 2)
    if key == ord("c"):
        imageFix = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        plt.imshow(imageFix)
        plt.show()
    # Show time to process whole frame
    frame_end = time.time()
    elapsed_time = frame_end-frame_start
    print("Frame processed in %.04f s (%02.2f frames per second)"%(elapsed_time, 1.0/elapsed_time))

# Close all windows
cv2.destroyAllWindows()
