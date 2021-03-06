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
ser = serial.Serial("/dev/ttyACM1", 115200, timeout=1)#, writeTimeout = 3)
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

def readNumbers():
    while (ser.inWaiting() > 0):
        try:
            line = ser.readline().decode("utf-8").rstrip()
            #print("serial output : ", line)
            print(line)
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
        return corners        
    else:
        #print("No Markers Found")
        return None
        
def findDist(corners):
   
    #Have to use 'if' again in case a marker isn't detected
    if corners is not None:
        #Focal Length is 3.60 mm & Pixel Size is 1.4 um x 1.4 um & Resolution is 1920 x 1080
        imageH = 480
        imageW = 640
        objH = 95
        objW = 95
        #focalL = 1841.36842105 #This is in pixels for 1920x1080 image
        focalL = 627.246315789 #This is in pixels for 640x480 video
        #widthA = 0
        
        #This Determines which half of the field of view the marker is in (left or right)
        #The absolute value is needed for a true comparison
        avgX = int((corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) /4)
        avgY = int((corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]) /4)
        perHeight = (abs(corners[0][0][0][1] - corners[0][0][3][1]) + abs(corners[0][0][1][1] - corners[0][0][2][1]))/2
        perWidth = (abs(corners[0][0][0][0] - corners[0][0][1][0]) + abs(corners[0][0][2][0] - corners[0][0][3][0]))/2
        #focalLength = perHeight * 647.7 / objH
        #print(focalLength)
        distToObj = focalL * objH / perHeight
        
        angleCave = avgX * 54 / imageW
        angleCave = 27 - angleCave #Positisve on the left & Negsative on the Right
        #widthA = widthA+(perWidth/2)        
#Just convert to different units to get correct values
        distToObj = distToObj / 10
        angleCave = (angleCave * 3.1415926535897932384626433832795 / 180) * 0.66
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
old_angle = 0
finalTime = 15
while(True):
    #Capture Frame by Frame
    ret, frame = cap.read()    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners = arucoDetection()
    object_location = findDist(corners)
    
    
    #This is for testing, slows down code
    if corners is not None:# and times <= finalTime:# and old_angle != object_location[0]):
        #if old_angle != object_location[0]:
        if times == finalTime or times == 0:
            writeNumbers(object_location)
        print(object_location[0])
        times += 1
        old_angle = object_location[0]
    #time.sleep(2)
    #print_LCD(object_location, corners)
        #print(object_location[0])

    #while True:
        
    #time.sleep(1)
    #while True:
     #   readNumbers()        
        #lcd.clear()
        #lcd.message = str(r)
        if times == finalTime+1:
            break
    #print(object_location)
    #Display the frame and kill prog if Q is pressed
#    cv2.imshow('frame', gray)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break

#Release Capture at End
cap.release()
#cv2.destroyAllWindows()
'''while True:
        
        # get set position from Aruco Marker
        set_pos = camera.markerDetection()
        #set_pos = 2
        #writes to the arduino where it needs to go
        if set_pos != None:
            writeNumber(set_pos)
        # sleep a tehnth of a second
        #time.sleep(1)
        # gets the actual position of the wheel from the encoder
        real_pos = readNumber()
        #prints it all to the LCD screen
        print_LCD(set_pos, real_pos)'''

