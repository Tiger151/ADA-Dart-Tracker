#Main Python program
import cv2
import math
import time
import os.path

from threading import Thread

# PRIMARY ADB Module 
# NOTE: Due to COVID-19 test were unable to performed. A placeholder pen throwing video is in
# use with the pen representing the dart.
# Therefore only one test with that video is possible. Given the video and parameters around
# the throw in the video, it will miss by -7.8ish meters below the target at the given distance 
# assuming the horizontal velocity is constant.   


#Pass to emulator variables
velocityX = None
velocityYintial = None

finalHorizontal = 0 #var to retain exit position for bottom cam

def SideCam():
    #Due to recording difficulties the Y axis on the camera will be using arbitrary distance between 2m and 0m. 0px = 2m, 1000px = 0m
    #X axis will be assumed to be 1m.
        
    #TOGGLE BETWEEN THE TEST VIDEO AND REAL CAMERA##########
    cap = cv2.VideoCapture("pen_repeat.m2t")
    #cap = cv2.VideoCapture(0)
    ########################################################
    #Frames before and after per 2 frames
    ret, frame0 = cap.read()
    ret, frame1 = cap.read()
    #cap.set(cv2.CAP_PROP_FPS, 60)
    ########################################################
    #print("==================CAMERAS==================")
    #FPS counter############################################
    fps = cap.get(cv2.CAP_PROP_FPS)
    fpsCounter = "SIDE CAM FPS: %s" % (fps)
    frameCount = 0
    print(fpsCounter)
    ########################################################
    
    enterBoxPos = None #For angle calculation
    exitBoxPos = None #For angle calculation
    
    positionPast = (0,0)#Init variable
    
    startTime = time.process_time()#timeout
    currentTime = None
    
    while cap.isOpened(): #while camera or video is open
        diffrence = cv2.absdiff(frame0, frame1)#record diffrence in frames
        grey = cv2.cvtColor(diffrence, cv2.COLOR_BGR2GRAY)#convert diffrence in frames to black&white for motion detection
        blur = cv2.GaussianBlur(grey, (5,5), cv2.BORDER_DEFAULT) #smooth frame to reduce imprefections of camera | higher values = more blur
        _, thresh = cv2.threshold(blur, 19, 255, cv2.THRESH_BINARY)#threshold = 19, set to 255 if >= 19, set to 0 is < 19.
        dilated = cv2.dilate(thresh, None, iterations = 4)#something to do with shape of object
        
        contor, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #get contours
        currentTime = time.process_time()
        
        for c in contor: #for each contour
            #if cv2.contourArea(c) >= 10:  #use this to filter out unwanted contours
            x,y,w,h = cv2.boundingRect(c)
            #forwardX = int(x + (w/2))
            forwardY = int(y + (h/2))
            #center = (forwardX,forwardY)
            dartForwardBottom = (x+w, y+h)
            dartForwardCenter = (x+w, forwardY)
            dartForwardTop = (x+w, y)
            
            frameCount = frameCount + 1
            
            #fps measurement
            positionNow = dartForwardBottom
            text = "FRAME[%s] past = %s now = %s" % (frameCount, positionPast, positionNow)
            print(text)
            positionPast = positionNow
            
            #Record now position as last
            exitBoxPos = positionNow
            
            #Record enter box pos
            if enterBoxPos is None:
                enterBoxPos = positionNow
                print("SIDE CAM: entered at", enterBoxPos)
            
            #center1 = (x,h)
            cv2.rectangle(frame0, (x,y), (x+w, y+h), (0,255,255), 3)
            cv2.circle(frame0, dartForwardBottom, 1, (255,60,255), 5)
            cv2.circle(frame0, dartForwardTop, 1, (255,60,255), 5)
            cv2.circle(frame0, dartForwardCenter, 1, (255,60,255), 5)
            #cv2.circle(frame0, (50,990), 1, (0,60,255), 5)
        #cv2.drawContours(frame0, contor, -1, (0,255,255), 3)
            
        cv2.imshow("side camera", frame0)
        
        frame0 = frame1
        ret, frame1 = cap.read()
        
        
        deltaTime = currentTime - startTime
    
        if deltaTime > 10: #10 second auto timeout
            break
        if cv2.waitKey(40) == 27: #PRESS ESCAPE TO END PROGRAM
            break
    
    print("SIDE CAM: entered at ", enterBoxPos, " Exited at ", exitBoxPos)
    
    #Trig to find theta
    opposite = None
    adjacent = None
    if exitBoxPos[1] > enterBoxPos[1]:
        #assume right angle of trig is below entry
        #calc sides
        opposite = exitBoxPos[1] - enterBoxPos[1]
        adjacent = exitBoxPos[0] - enterBoxPos[0]
        theta = math.degrees(math.atan((opposite/adjacent)))
    else:
        #assume right angle of trig is above entry on other side
        #calc sides
        adjacent = exitBoxPos[0] - enterBoxPos[0]
        opposite = exitBoxPos[1] - enterBoxPos[1]
        theta = math.degrees(math.atan((opposite/adjacent)))
        
    
    #Velocity Horizontal
    global velocityX
    realFPS = round(fps, 0) #round to avoid inaccuracy with software
    #oneSecond = (frameCount/realFPS) * (realFPS/frameCount)
    velocityX = 1 * (realFPS/frameCount) #1m view horizontal distance
    #viewable X distance = 1m so how ever many frames / frames per second = 1m per second
    
    #Velocity Veritcal (This will be off a little)
    global velocityYintial
    #2m = 1000px   1
    enterYtoMeters = (enterBoxPos[1] / 500)
    exitYtoMeters = (exitBoxPos[1] / 500)
    velocityYintial = ((enterYtoMeters + 1) - exitYtoMeters) #in meters 
    
    print("SIDE CAM: VelocityX = 1m per", velocityX, "sec")
    print("SIDE CAM: Theta", theta)
    print("SIDE CAM: Real FPS", realFPS)
    #Garabage disposal#######################################
    cv2.destroyAllWindows()
    #cv2.release()

def BottomCam():
    #Note this is reusing the same video for simplicty
    #The Y axis represents the horizontal direction the ADB must move. (Only for video testing puropses)
    
    #TOGGLE BETWEEN THE TEST VIDEO AND REAL CAMERA##########
    cap = cv2.VideoCapture("pen_repeat.m2t")
    #cap = cv2.VideoCapture(0)
    ########################################################
    #Frames before and after per 2 frames
    ret, frame0 = cap.read()
    ret, frame1 = cap.read()
    #cap.set(cv2.CAP_PROP_FPS, 60)
    ########################################################
    
    ########################################################
    
    enterBoxPosX = None
    exitBoxPosX = None
    
    startTime = time.process_time()#timeout
    currentTime = None
    while cap.isOpened(): #while camera or video is open
        diffrence = cv2.absdiff(frame0, frame1) #record diffrence in frames
        grey = cv2.cvtColor(diffrence, cv2.COLOR_BGR2GRAY) #convert diffrence in frames to black&white for motion detection
        blur = cv2.GaussianBlur(grey, (5,5), cv2.BORDER_DEFAULT) #smooth frame to reduce imprefections of camera | higher values = more blur
        _, thresh = cv2.threshold(blur, 19, 255, cv2.THRESH_BINARY) #threshold = 19, set to 255 if >= 19, set to 0 is < 19.
        dilated = cv2.dilate(thresh, None, iterations = 4) #something to do with shape of object
        
        contor, _ = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #get contours
        
        currentTime = time.process_time()
        for c in contor: #for each contour
            #if cv2.contourArea(c) >= 10:  #use this to filter out unwanted contours
            x,y,w,h = cv2.boundingRect(c)
            #forwardX = int(x + (w/2))
            forwardY = int(y + (h/2))
            #center = (forwardX,forwardY)
            dartForwardBottom = (x+w, y+h)
            dartForwardCenter = (x+w, forwardY)
            dartForwardTop = (x+w, y)
            
            positionNow = dartForwardBottom
            
            #positionPast = positionNow
            
            #Record now position as last
            exitBoxPosX = positionNow
            
            #Record enter box pos
            if enterBoxPosX is None:
                enterBoxPosX = positionNow
                print("BOTTOM CAM: entered at ", enterBoxPosX)
            
            cv2.rectangle(frame0, (x,y), (x+w, y+h), (0,255,255), 3)
            cv2.circle(frame0, dartForwardBottom, 1, (255,60,255), 5)
            cv2.circle(frame0, dartForwardTop, 1, (255,60,255), 5)
            cv2.circle(frame0, dartForwardCenter, 1, (255,60,255), 5)
            
        cv2.imshow("bottom camera", frame0)
        
        frame0 = frame1
        ret, frame1 = cap.read()
        
        global finalHorizontal
        finalHorizontal = exitBoxPosX #gustimate for simplyfing curved throw on the ADB X axis
        
        deltaTime = currentTime - startTime
    
        if deltaTime > 10: #10 second auto timeout
            break
        if cv2.waitKey(40) == 27: #PRESS ESCAPE TO END PROGRAM
            break
    print("BOTTOM CAM: entered at ", enterBoxPosX, " Exited at ", exitBoxPosX)
    
    #garbage
    cv2.destroyAllWindows()

#Embeded Emulator (SEE Emulator.py for documentation)
class Emulator:
    gx = 0
    gy = 0
    motorLeft = 0
    motorRight = 0
    motorUpDown = 0
    prevX = 0
    prevY = 0
    def __init__(self, x, y):
        self.gx = x
        self.gy = y
        if (os.path.isfile("positions.csv")):
            file = open("positions.csv", "r")
            items = file.readline().split(',')
            file.close()
            self.motorLeft = int(items[0])
            self.motorRight = int(items[1])
            self.motorUpDown = int(items[2])
            self.prevX = int(items[3])
            self.prevY = int(items[4])
            
            if self.gy > 50:
                self.gy = 50
            if self.gy < -50:
                self.gy = -50
            if self.gx > 50:
                self.gx = 50
            if self.gx < -50:
                self.gx = -50
		
    def findMoveAmmount(self, currentPos, gotoPos):
        return (-(currentPos) + gotoPos);

    def spinUp(self, howMuch):
        self.motorUpDown = howMuch
        return;
    
    def spinDown(self, howMuch):
        self.motorUpDown = howMuch
        return;
    
    def spinLeft(self, howMuch):
        self.motorLeft = howMuch
        return;
        
    def spinRight(self, howMuch):
        self.motorRight = howMuch
        return;
    
    def run(self):
        xMove = self.findMoveAmmount(self.prevX, self.gx)
        yMove = self.findMoveAmmount(self.prevY, self.gy)
        
        #y axis move
        if yMove > 0:
            self.spinUp(yMove)
        if yMove < 0:
            self.spinDown(yMove)
        #x axis move
        if xMove > 0:
            self.spinRight(xMove)
        if xMove < 0:
            self.spinLeft(xMove)
        print("Moved X:%d Y:%d" % (xMove, yMove))
        print("POS X:%d Y:%d" % (self.gx, self.gy))
      
        file = open("positions.csv", "w")
        file.write(str(self.motorLeft))
        file.write(",")
        file.write(str(self.motorRight))
        file.write(",")
        file.write(str(self.motorUpDown))
        file.write(",")
        file.write(str(xMove))
        file.write(",")
        file.write(str(yMove))
        file.close()

#end of embedded emulator


print("==================CAMERAS START==================")
#Multi threading to run 2 camera instances at same time with parallelism
sideT = Thread(target = SideCam)
bottomT = Thread(target = BottomCam)
sideT.start()
bottomT.start()
sideT.join()
bottomT.join()
print("===================CAMERAS END===================")

#physics calculations
#vertical height @ adb position = (virtical velocity)t-(1/2)gt^2
#t = (virtical horizontal)/total distance            1m + 3m = 4m
#velocityY = (intialVelocity) - gt
t = velocityX / 4
velocityY = velocityYintial - 9.81 * t
finalHeight = (velocityY * t) - (1/2) * 9.81 * (t**2)
#finalHeight + 1m offset for ABD 1m above ground
finalHeight = finalHeight + 1

isMiss = False

if finalHeight < 1 or finalHeight > 2: #outside of ADB movement range
    isMiss = True
elif finalHorizontal[1] < 250 or finalHorizontal[1] > 750: #between 250 - 750 (aka 1m)
    isMiss = True
    
    
isMiss = False #toggle test

if not isMiss: #inside ADB Y and X movement (Assuming adb can go )
    #convert finalHorizontal to point for X coord
    #between 250-750 (2m) converted to 0-500 (1m) by subtracting 250 because: 250(1/2m) + 500(1m) + 250(1/2m) = 1000(2m)  second 250 is ignored as it does not fall in range.
    #1% of 500 = 500/100 = 5
    #(finalHorizontal / 5) allows for 0 to 100 range conversion
    # -50 for negative range on emulator/ADB
    #must round to nearest whole integer
    
    #((finalHorizontal - 250) / 5) - 50
    sendX = round(((finalHorizontal[1] - 250) / 5) - 50) #horizontal
    #print(sendX)
    #print(finalHeight)
    
    #finalHeight
    #test = 1.3
    #convert finalHeight to point for Y coord
    #one 500th = 0.002
    sendY = round((((finalHeight - 1) / 0.002) / 5) - 50) #virtical
    #print(sendY)
    #pass data onto emulator
    emu = Emulator(sendX, sendY)
    emu.run()