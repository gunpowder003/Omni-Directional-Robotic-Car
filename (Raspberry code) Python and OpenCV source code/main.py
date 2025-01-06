import cv2
import cv2.cv as cv
import numpy as np
import serial
import struct
import time
time.sleep(1)

#initializing serial port: use /dev/tty/ACM1 @ /dev/tty/ACM0 @ /dev/ttyUSB0 @ /dev/ttyUSB1 
arduino = serial.Serial('/dev/ttyACM0', 115200,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout = 1)
                        

kernel = np.ones((5,5),np.uint8)

# Take input from webcam
cap = cv2.VideoCapture(-1)

# Reduce the size of video to 320x240 so rpi can process faster
cap.set(3,320)
cap.set(4,240)


def nothing(x):
    pass
# Creating a windows for later use
cv2.namedWindow('HueComp')
cv2.namedWindow('SatComp')
cv2.namedWindow('ValComp')
cv2.namedWindow('closing')
cv2.namedWindow('tracking')



# Creating track bar for min and max for hue, saturation and value
# You can adjust the defaults as you like
cv2.createTrackbar('hmin', 'HueComp',80,255,nothing)
cv2.createTrackbar('hmax', 'HueComp',106,255,nothing)

cv2.createTrackbar('smin', 'SatComp',85,255,nothing)
cv2.createTrackbar('smax', 'SatComp',1931,255,nothing)

cv2.createTrackbar('vmin', 'ValComp',162,255,nothing)
cv2.createTrackbar('vmax', 'ValComp',255,255,nothing)

# tested value for IIUM green jacket #normal light #low light #ruang tamu bilik
# hmn = 92 #82      68
# hmx = 102 #90     157
# smn = 117 #117    30
# smx = 255 #255    101
# vmn = 0           112
# vmx = 255         255


while(1):

    buzz = 0
    _, frame = cap.read()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    hue,sat,val = cv2.split(hsv)

    # get info from track bar and appy to result
    hmn = cv2.getTrackbarPos('hmin','HueComp')
    hmx = cv2.getTrackbarPos('hmax','HueComp')
    

    smn = cv2.getTrackbarPos('smin','SatComp')
    smx = cv2.getTrackbarPos('smax','SatComp')


    vmn = cv2.getTrackbarPos('vmin','ValComp')
    vmx = cv2.getTrackbarPos('vmax','ValComp')

    # Apply thresholding
    hthresh = cv2.inRange(np.array(hue),np.array(hmn),np.array(hmx))
    #hthresh = cv2.GaussianBlur(hthresh,(5,5),0)
    #hthresh = cv2.morphologyEx(hthresh, cv2.MORPH_OPEN, kernel)
    sthresh = cv2.inRange(np.array(sat),np.array(smn),np.array(smx))
    vthresh = cv2.inRange(np.array(val),np.array(vmn),np.array(vmx))

    # AND h s and v
    tracking = cv2.bitwise_and(hthresh,cv2.bitwise_and(sthresh,vthresh))

    # Some morpholigical filtering
    dilation = cv2.dilate(tracking,kernel,iterations = 1)
    closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    closing = cv2.GaussianBlur(closing,(5,5),0)
    closing = cv2.erode(closing,kernel,iterations = 1)
    #closing = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)

    gray = closing
    ret,gray = cv2.threshold(gray,127,255,0)
    gray2 = gray.copy()
    mask = np.zeros(gray.shape,np.uint8)

    contours, hier = cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        minArea = 4000 #minimum contour size to be detected
        maxArea = 50000 #maximum contour size to be detected
        
        if minArea<cv2.contourArea(cnt)<maxArea:
            #draw rectangle around contour
            x,y,w,h = cv2.boundingRect(cnt)
            #draw curve around bounded contour
            cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
            cv2.drawContours(frame,[cnt],0,(0,255,0),2)
            #cv2.drawContours(mask,[cnt],0,255,-1)
            #calculate contour area and rectangle position
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)
            x = int(M['m10']/M['m00']) #x axis position
            
            speedy = 1-(2*(area-minArea)/(maxArea-minArea))
            #sending y direction speed to arduino
            speedy = int(float(speedy*1024))
            y = speedy
            
            #speedy = str(speedy)
            #arduino.write(speedy)
            #print(y)
            '''
            if area==0:
                arduino.write('m')
            if 0 < x < 140  or 180 < x < 320:
                if x<=140:
                    arduino.write('i')
                elif x>=180:
                    arduino,write('k')
            elif  140 <= x <= 180:
                if 0 < y <100:
                    arduino.write('s')
                elif 100 <= y < 600:
                    arduino.write('a')
                elif 600 <= y < 1024:
                    arduino.write('b')
                elif -300 <= y < 0:
                    arduino.write('e')
                elif -1024 <= y < -300:
                    arduino.write('f')
                    
            '''
            if area==0:
                arduino.write('m')
            #pointing left and right command
            if 350 <= y <1024 or y <= 300:
                if 140 <= x < 180:
                    if 350 <= y < 600:
                        arduino.write("a"*400)
                    elif 600 <= y < 1024:
                        arduino.write("b"*400)
                    elif 0 <= y <=250:
                        arduino.write("e" *400)
                    elif -1024 <= y < 0:
                        arduino.write("f" *400)
                    '''arduino.write('s')'''
                elif 180 <= x < 250:
                   arduino.write("k" *250)
                elif 250 <= x < 320:
                    arduino.write("l" *250)
                elif 60 <= x < 140:
                    arduino.write("i"*250)
                elif 0 <= x < 60:
                    arduino.write("j"*250)
             #moving forward ,backward and strafe command   
            elif x>180 or x<140:
                if 300 < y < 350:
                    if 140 <= x < 180:
                        arduino.write("s"*100)
                    elif 180 <= x < 250:
                        arduino.write("c"*100)
                    elif 250 <= x < 320:
                        arduino.write("d"*100)
                    elif 60 <= x < 140:
                        arduino.write("g"*100)
                    elif 0 <= x < 60:
                        arduino.write("h"*100)
                        '''
                elif 350 <= y < 600:
                    arduino.write('a')
                elif 600 <= y < 1024:
                    arduino.write('b')
                elif 0 <= y <=250:
                    arduino.write('e')
                elif -1024 <= y < 0:
                    arduino.write('f')
                    '''
            
    
    #Show the result in frames
    cv2.imshow('HueComp',hthresh)
    cv2.imshow('SatComp',sthresh)
    cv2.imshow('ValComp',vthresh)
    cv2.imshow('closing',closing)
    cv2.imshow('tracking',frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()
