import argparse as ap
import os.path

#ADA Motor emulator
#   X X X | X X X
#   X X X | X X X
#   X X X | X X X
#   ------0------
#   X X X | X X X
#   X X X | X X X
#   X X X | X X X
#assuming -50 to 50 X/Y movement grid starting at 0,0
#each numbered unit represents 1 revolution with the spindal attached
#3 motors (1: up/down, 1:left, 1:right)
#Motors left and right move togeather while the up/down motor is independent with gravity assisted drop.

#############################################
#ARGUMENTS:
# argument x: the goto horizontal location in terms of -50 to 50
# argument y: the goto vertical location in terms of -50 to 50
# see Main.py for conversion of exit camera pixels to meters to these units
#############################################

#inital vars
#load values from last throw(emulates previous throw position)
motorLeft = 0
motorRight = 0
motorUpDown = 0
prevX = 0
prevY = 0

sendX = 0
sendY = 0

if (os.path.isfile("positions.csv")):
    file = open("positions.csv", "r")
    items = file.readline().split(',')
    #print("Previous Move L:%s R:%s U/D:%s" % (items[0], items[1], items[2]))
    file.close()
    motorLeft = int(items[0])
    motorRight = int(items[1])
    motorUpDown = int(items[2])
    prevX = int(items[3])
    prevY = int(items[4])
#else:
    #print("Previous Move L:0 R:0 U/D:0")

#handle arguments
parser = ap.ArgumentParser(description="emulator for motor control")
parser.add_argument('x', help="X pos to goto", type=int)
parser.add_argument('y', help="Y pos to goto", type=int)
args = parser.parse_args()

sendX = args.x
sendY = args.y

#stop motors if x or y is greather than 50 or less than -50
if sendY > 50:
    sendY = 50
if sendY < -50:
    sendY = -50
if sendX > 50:
    sendX = 50
if sendX < -50:
    sendX = -50

#used to find move ammount between the current position and the goto position for either axis
def findMoveAmmount(currentPos, gotoPos):
        return (-(currentPos) + gotoPos);

#emulate revolution
def spinUp(howMuch):
    global motorUpDown
    motorUpDown = howMuch
    return;

def spinDown(howMuch):
    global motorUpDown
    motorUpDown = howMuch
    return;

def spinLeft(howMuch):
    global motorLeft
    motorLeft = howMuch
    #motorright = args.x =    #must spin oposite motor reverse when horizontal
    return;
    
def spinRight(howMuch):
    global motorRight
    motorRight = howMuch
    #motorLeft = -(howMuch)               #must spin oposite motor reverse when horizontal
    return;

xMove = findMoveAmmount(prevX, sendX)
yMove = findMoveAmmount(prevY, sendY)

#y axis move
if yMove > 0:
    spinUp(yMove)
if yMove < 0:
    spinDown(yMove)
#x axis move
if xMove > 0:
    spinRight(xMove)
if xMove < 0:
    spinLeft(xMove)
print("Moved X:%d Y:%d" % (xMove, yMove))
print("POS X:%d Y:%d" % (sendX, sendY))
#save current positions to file(emulates halted after throw) 
file = open("positions.csv", "w")
file.write(str(motorLeft))
file.write(",")
file.write(str(motorRight))
file.write(",")
file.write(str(motorUpDown))
file.write(",")
file.write(str(sendX))
file.write(",")
file.write(str(sendY))
file.close()