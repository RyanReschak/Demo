'''
Ryan Reschak and Doyle Smith
Uses Serial to send a image taken integer to the Arduino
Ardunio spits back wheel position
Make sure to check the connections.
'''
#communication
import serial
import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import math
import cv2
import numpy as np
#Arcuo Detection Imports
from cv2 import aruco
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)#, writeTimeout = 3)
ser.flush()
# This is the address we setup in the Arduino Program
address = 0x04
offset = 1
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 120) #Absolute Magic

#camera.balanceExposure()

def writeNumbers(object_location):
    try:
        
        ang_pos = str(round(object_location[0], 3)) + ' ' + str(round(object_location[1],3)) + '\n'
        #print(ang_pos)
        val_bytes = ang_pos.encode('utf-8')
        #val = bytes(str(object_location[0]) + ' ' + str(object_location[1]) + '\n', 'utf-8')
        #val = b"1\n"#'1\n', 'utf-8')
        #val = b'' + 
        #val = bytes(str(object_location[0]) + '\n', 'utf-8')
        ser.write(val_bytes)
    except IOError:
        print("Can't Write to Arduino")
        return "error_w"

def readSerial():
    while (ser.inWaiting() > 0):
        try:
            line = ser.readline().decode("utf-8").rstrip()
            print("serial output : ", line)
            return line
    
        except:
            print("Communication Error")

def print_LCD(object_location, corners):
    try:
        

        lcd.clear()
        #if statement needed for set pos and real pos
#        if (corners is None):
#            #print("Marker Not Found")
#            #lcd.message = "Maker Not\nDetected!"
#        else:
#            #print("Marker Found")
#            #lcd.message = "Maker Detected!\nAngle:" + str(object_location[1])
    except IOError:
        print("Can't write to LCD\n")

def arucoDetection():
    #Aruco Detection#
    #detects what marker # it is
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    #plt.figure()
    #plt.imshow(frame_markers)
    #This is for not erroring out if there is not an image in the photo
    if ids is not None:
       # print("Marker Found")
#        for i in range(len(ids)):
#            c = corners[i][0]
#            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
    #        plt.legend()
        #plt.show()
    #        print(ids)
        return [corners, ids]        
    else:
        #print("No Markers Found")
        return [None, None]
        
def findDist(corners):
   
    #Have to use 'if' again in case a marker isn't detected
    if corners is not None:
        #Focal Length is 3.60 mm & Pixel Size is 1.4 um x 1.4 um & Resolution is 1920 x 1080
        imageH = 480
        imageW = 640
        objH = 100
        objW = 100
        #focalL = 1841.36842105 #This is in pixels for 1920x1080 image
        focalL = 627.246315789 #This is in pixels for 640x480 video
        #widthA = 0
        
        #This Determines which half of the field of view the marker is in (left or right)
        #The absolute value is needed for a true comparison
        avgX = int((corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0]) /4)
        avgY = int((corners[0][0][1] + corners[0][1][1] + corners[0][2][1] + corners[0][3][1]) /4)
        perHeight = (abs(corners[0][0][1] - corners[0][3][1]) + abs(corners[0][1][1] - corners[0][2][1]))/2
        perWidth = (abs(corners[0][0][0] - corners[0][1][0]) + abs(corners[0][2][0] - corners[0][3][0]))/2
        #focalLength = perHeight * 647.7 / objH
        #print(focalLength)
        distToObj = focalL * objH / perHeight
        
        angleCave = avgX * 54 / imageW
        angleCave = 27 - angleCave #Positisve on the left & Negsative on the Right
        #widthA = widthA+(perWidth/2)        
#Just convert to different units to get correct values
        distToObj = distToObj / 10
        angleCave = (angleCave * 3.1415926535897932384626433832795 / 180) * 0.8
        #print(distToObj," mm")
        #print(distToObj*0.0393," in")
        #print(angle, " degrees")
        return [angleCave, distToObj]

    else:
        return None
        #print("No Markers Found")

lcd.clear()
# Set LCD color to red
lcd.color = [100, 0, 255]
# grab a video from the camera
times = 0
#old_angle = 0
finalTime = 15
markers_order = [0,1,2,3,4,5]
index_order = 0 

while(True):
    
    #Use camera with fast shutter speed + high brightness + contrast
    #Capture Frame by Frame
#    tme = []
#    for i in range(100):
#        t = time.time()
#        ret, frame = cap.read()
#        elapsed = time.time() - t
#        tme.append(elapsed)
#    print(sum(tme)/len(tme))
    
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids = arucoDetection()
    
    if corners is not None:
        #this is the index the id is on
        found_index = None
        #find beacon 
        for i in range(len(corners)):
            if (ids[i][0] == markers_order[index_order]):
                found_index = i
        if found_index is None:
            continue
        #print(corners)
        #print(corners[found_index])
        object_location = findDist(corners[found_index])
    
        #if old_angle != object_location[0]:
        if times >= finalTime or times == 0:
            print(object_location[0])
            #print(corners[found_index])
            writeNumbers(object_location)
            if times == finalTime:
                index_order += 1
                time.sleep(0.2)
                while (readSerial() != "Y"):
                    continue
                #incase there is a problem
                times = 0
                if index_order == len(markers_order):
                    time.sleep(10)
                    break
                
                continue
        
        #print(ids[0][0])
        times += 1
        #old_angle = object_location[0]
        
    #print(object_location)
    #Display the frame and kill prog if Q is pressed
#    cv2.imshow('frame', gray)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
ser.close()
#Release Capture at End
cap.release()
#cv2.destroyAllWindows()


