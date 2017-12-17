# -*- coding: utf-8 -*-
"""
Created on Sat Nov 25 12:48:42 2017

@author: Anabel RM
"""

import numpy as np
import math
import cv2
import serial
 
ser = serial.Serial('COM15', 9600) #9600 baud
cap = cv2.VideoCapture(0)
lowerGreen = np.array([49,50,50], dtype=np.uint8)        
upperGreen = np.array([80, 255, 255], dtype=np.uint8)
kernel=np.ones((5,5),np.uint8)
x=1
y=1
w=1
h=1
oldError=0
ierror=0
derror=0
ref=0
# Choose an initialization parameter vector
p = [0, 0, 0]
# Define potential changes
dp = [1, 1, 1]
# Calculate the error

output=0
while(True):
    
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV) 
    mask = cv2.inRange(hsv, lowerGreen, upperGreen)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    momentsG = cv2.moments(mask)
    area = momentsG['m00']

    opening=cv2.morphologyEx(mask, cv2.MORPH_OPEN,kernel)
    threshold = 0.00001
    
    if(area > 2000000):
     
        #centro x, y
        x = int(momentsG['m10']/momentsG['m00'])
        y = int(momentsG['m01']/momentsG['m00'])
        #print ("x = ", x,"y = ", y)
        (x, y, w, h) = cv2.boundingRect(opening)
        cv2.rectangle(frame, (x, y), (x+w, y+h),(0,0,255), 2)
        posGreen=np.array([x,y,x+w, y+h], dtype=np.uint8)
        angleH=math.atan(h/w)*180/math.pi
        #ob.getAngle(angleH)
        angleV=math.atan((w)/(h))*180/math.pi
        print ("angle Horizontal = ", angleH)
        print ("angle Vertical = ", angleV)
        best_err = angleV
        
        
        #-----------PID----------------
        #err = angleH - ref
        #ierror +=err
        #derror = err-oldError
        #output = - p[0]*err - p[1]*ierror - p[2]*derror
        #output =  (0.3276*err + 0.0088*ierror + 0.1*derror)+1000
        #oldError = err
        #output=int(output)
        angleH=int(angleH)
        #print("output",output)
        #for i in range(output):
        #ser.write("A".encode())
        #    print("yes")
        #    i+=20
        
        move=list(map(int, str(angleH)))
        
        n=move[0]
        if(angleH<10):
            ser.write("A".encode())
        elif(n==1):
            ser.write("B".encode())
        elif(n==2):
            ser.write("C".encode())
        elif(n==3):
            ser.write("D".encode())
        elif(n==4):
            ser.write("E".encode())
        elif(n==5):
            ser.write("F".encode())
        elif(n==6):
            ser.write("G".encode())
        elif(n==7):
            ser.write("H".encode())
        elif(n==8):
            ser.write("I".encode())
        elif(n==9):
            ser.write("J".encode())

        print(n)
        """
        #------- END PID-------------
        #move(OUTPUT , 1);
        
        #while sum(dp) > threshold:
        if sum(dp) > threshold:
            for i in range(len(p)):
                p[0] += dp[0]
    
                if err < best_err:  # Hubo mejoras
                    best_err = err
                    dp[0] *= 1.1
                else:  # No hubo mejora
                    p[0] -= 2*dp[0]  # Cambio de direccion
                        
                    if err < best_err:  # hubo mejora
                        best_err = err
                        dp[0] *= 1.05
                    else: # No hubo mejora
                        p[0] += 1.01*dp[0]
                        # No hubo mejora, puede ser porque el delta era muy alto
                        dp[0] *= 0.95
        
        
        print(p)
        """

    cv2.imshow('Camara', frame)
    cv2.imshow('maskGreen', mask)
    cv2.imshow('res', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows();