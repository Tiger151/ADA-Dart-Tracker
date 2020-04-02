import cv2
import numpy as np

#TOGGLE BETWEEN THE TEST VIDEO AND REAL CAMERA##########
cap = cv2.VideoCapture("pen_repeat.m2t")
#cap = cv2.VideoCapture(0)
########################################################
#Frames before and after per 2 frames
ret, frame0 = cap.read()
ret, frame1 = cap.read()
#cap.set(cv2.CAP_PROP_FPS, 60)
########################################################
#FPS counter############################################
fps = cap.get(cv2.CAP_PROP_FPS)
fpsCounter = "FPS: %s" % (fps)
print(fpsCounter)
########################################################

positionPast = (0,0)#Init variable
while cap.isOpened(): #while camera or video is open
    diffrence = cv2.absdiff(frame0, frame1)#record diffrence in frames
    grey = cv2.cvtColor(diffrence, cv2.COLOR_BGR2GRAY)#convert diffrence in frames to black&white for motion detection
    blur = cv2.GaussianBlur(grey, (5,5), cv2.BORDER_DEFAULT) #smooth frame to reduce imprefections of camera | higher values = more blur
    _, thresh = cv2.threshold(blur, 19, 255, cv2.THRESH_BINARY)#threshold = 19, set to 255 if >= 19, set to 0 is < 19.
    dilated = cv2.dilate(thresh, None, iterations = 4)#something to do with shape of object
    
    contor, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #get contours
    
    for c in contor: #for each contour
        #if cv2.contourArea(c) >= 10:  #use this to filter out unwanted contours
        x,y,w,h = cv2.boundingRect(c)
        forwardX = int(x + (w/2))
        forwardY = int(y + (h/2))
        #center = (forwardX,forwardY)
        dartForwardBottom = (x+w, y+h)
        dartForwardCenter = (x+w, forwardY)
        dartForwardTop = (x+w, y)
        
        
        #fps measurement
        positionNow = dartForwardBottom
        text = "past = %s now = %s" % (positionPast, positionNow)
        print(text)
        positionPast = positionNow
        
        #center1 = (x,h)
        cv2.rectangle(frame0, (x,y), (x+w, y+h), (0,255,255), 3)
        cv2.circle(frame0, dartForwardBottom, 1, (255,60,255), 5)
        cv2.circle(frame0, dartForwardTop, 1, (255,60,255), 5)
        cv2.circle(frame0, dartForwardCenter, 1, (255,60,255), 5)
        #cv2.circle(frame0, center1, 1, (0,60,255), 5)
    #cv2.drawContours(frame0, contor, -1, (0,255,255), 3)
        
    cv2.imshow("image", frame0)
    
    frame0 = frame1
    ret, frame1 = cap.read()
    
    if cv2.waitKey(40) == 27: #PRESS ESCAPE TO END PROGRAM
        break
#Garabage disposal#######################################
cv2.destroyAllWindows()
#cv2.release()