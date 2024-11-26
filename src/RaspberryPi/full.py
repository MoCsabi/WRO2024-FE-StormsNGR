#! /usr/bin/python
'''The main python file, it contains the code for both robot runs and helper functions'''
import json
from math import asin, cos, pi, atan, ceil, copysign, floor, sin, sqrt, tan
import os
import random
import threading
from time import sleep, time
from log import *
from LedAndKey import *
from ESP32_Service import *
import LidarService
from LidarService import *
from Buzzer import *
from utilities import func_thread
import time

class refr_dict(dict):
    _temp = None
    
    def __init__(self,*args):
        super().__init__()
        self.update(*args)
    
    def _update_(self,*args):
        super().update(*args)
    
    def update(self,*args):
        refr_dict._temp = self
        for i in args:
            exec(f"refr_dict._temp._update_({{'{i}':(lambda: {i})}})")
        
    def __getitem__(self, key):
        item = super().__getitem__(key)
        return item()
    
    def values(self):
        return [ i() for i in super().values()]
    
    def items(self):
        return [(i[0],i[1]()) for i in super().items()]
    
    def __str__(self) -> str:
        return "{" + ", ".join([ ": ".join([str(i[0]),str(i[1])]) for i in self.items()]) + "}"
    


# distSensor=VL53L1X()
# '''Laser distance sensor behind the robot'''

TICKS_PER_CM=51.46
'''Convert internal ticks to CM. (Motor rotation is measured in ticks by the ESP)'''

direction:int=1
'''Stores the detected randomized direction during obstacle challenge. 
1: robot turns right
-1: robot turns left'''

GREEN=1
'''Constant for the color Green. Used for obstacle color detection and storing.'''
RED=2
'''Constant for the color Red. Used for obstacle color detection and storing.'''
import pixy
# TODO: Pixy camera currently not in use

colorMatrixPrequel = [RED,GREEN]
getColorCounter = 1


CAM_WIDTH=315
CAM_HEIGHT=207
CAM_RIGHT_CUTOFF=CAM_WIDTH-0 #200
CAM_LEFT_CUTOFF=50
CAM_BOT_CUTOFF=CAM_HEIGHT-10
CAM_TOP_CUTOFF=0
def isInsideCamArea(x,y):
    return True #currently camera cutoff is not in use
    return x<CAM_RIGHT_CUTOFF and x>CAM_LEFT_CUTOFF and y>CAM_TOP_CUTOFF and y<CAM_BOT_CUTOFF

defaultColor=RED
''''''
def checkColor()->int:
    '''Checks the color of the closest obstacle outside the cutoff area using the Pixy camera.'''
    global getColorCounter
    global CAM_RIGHT_CUTOFF
    global CAM_LEFT_CUTOFF
    global pixyObjects
    if lane==0:
        CAM_LEFT_CUTOFF=90
        CAM_RIGHT_CUTOFF=240
    elif lane>0:
        CAM_LEFT_CUTOFF=0
        CAM_RIGHT_CUTOFF=CAM_WIDTH-150
    elif lane<0:
        CAM_LEFT_CUTOFF=150
        CAM_RIGHT_CUTOFF=CAM_WIDTH
    # getColorCounter += 1
    # pixyObjects.append((110+random.randint(0,100),120+random.randint(-10,10),10+random.randint(-2,5),20+random.randint(-2,5),colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)],True))
    # log.info("color: %s"%colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)])
    # return colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)]
    temp_pixyObjects=[]
    blocks=pixy.BlockArray(4)
    n=pixy.ccc_get_blocks(4,blocks)
    log.info("detected %s blocks"%n)
    if n==0:
        log.error("No blocks detected!")
        pixyObjects=temp_pixyObjects
        return defaultColor
    for b in range(n): temp_pixyObjects.append((blocks[b].m_x,blocks[b].m_y,blocks[b].m_width,blocks[b].m_height,blocks[b].m_signature,False))
    for i in range(n):
        b:pixy.Block=blocks[i]
        log.debug("detected at %s %s, size %s %s with signature: %s"%(b.m_x,b.m_y,b.m_width,b.m_height,b.m_signature))
        if isInsideCamArea(b.m_x,b.m_y):
            log.info("i %s"%i)
            detected=list(temp_pixyObjects[i])
            detected[5]=True
            temp_pixyObjects[i]=tuple(detected)
            log.info("Final color %s"% "GREEN" if b.m_signature==GREEN else "RED")
            pixyObjects=temp_pixyObjects
            return b.m_signature
    else:
        log.error("All blocks outside cutoff range!! cutoffs: left %s right %s top %s bot %s"%(CAM_LEFT_CUTOFF,CAM_RIGHT_CUTOFF,CAM_TOP_CUTOFF,CAM_BOT_CUTOFF))
        pixyObjects=temp_pixyObjects
        return defaultColor


wIntegral=0
'''Wall distance following integral variable'''
wLastError=0
'''Wall distance following last error variable for derivative'''
wkP=1
'''Wall distance following proportional constant'''
wkI=0.1
'''Wall distance following integral constant'''
wkD=0.5
'''Wall distance following derivative constant'''
wallTarget=50
'''Wall distance following target distance in centimeters'''
pilotHeadingTarget:int=0
'''Target robot direction'''
lastHeading=0
'''Last heading direction recorded'''
heading0=0
'''0 heading offset'''

lastTOFDist=0

PILOT_NONE=0
'''Robot piloting mode, no piloting'''
PILOT_FOLLOW_LEFT=1
'''Robot piloting mode, follow left wall at wallTarget'''
PILOT_FOLLOW_RIGHT=2
'''Robot piloting mode, follow right wall at walltarget'''

pilotMode:int=0
'''Current pilot mode'''

def setPilotMode(pilotModeIn:int, param:int=10000):
    '''Set pilotMode and wallTarget if needed'''
    global pilotMode
    global wallTarget
    pilotMode=pilotModeIn
    if pilotModeIn==PILOT_FOLLOW_LEFT or pilotModeIn==PILOT_FOLLOW_RIGHT: wallTarget=param

def go(speed:int,headingTarget:int, pilotModeIn:int=PILOT_NONE, wallDistance:int=-1):
    '''Starts the robot with given parameters

    speed: speed of the robot measured in ticks/second. (0-3000)<br>
    headingTarget: Target direction in degrees (-90 - 90)<br>
    pilotMode: If left at default value no piloting, otherwise set given pilotmode (PILOT_FOLLOW_LEFT, PILOT_FOLLOW_RIGHT)<br>
    wallDistance: Only relevant if pilotMode is not PILOT_NONE, sets target wall distance for wall following'''
    global pilotHeadingTarget
    global wIntegral
    log.debug("go with speed: %s headingT: %s pilotMode: %s walldist: %s"%(speed,headingTarget,pilotModeIn,wallDistance))
    #reset wall following integral variable to ensure no previous buildup is kept
    wIntegral=0
    setTargetSpeed(speed)
    setPilotMode(pilotModeIn,wallDistance)
    pilotHeadingTarget=headingTarget
    setHeadingTarget(headingTarget)
    #
    setSteerMode(SMODE_GYRO)
    if speed>0:
        setVMode(VMODE_FORWARD)
    else:
        setVMode(VMODE_BACKWARD)

def goUnreg(power:int,headingTarget:int):
    '''Starts the motors without PID speed control, instead using constant power

    power: The constant power (-100 - 100)<br>
    headingTarget: Target direction in degrees (-90 - 90)'''
    global pilotHeadingTarget
    setUnregulatedPower(power)
    pilotHeadingTarget=headingTarget
    setHeadingTarget(headingTarget)
    setSteerMode(SMODE_GYRO)
    setVMode(VMODE_UNREGULATED)

def angularToXy(angle:int, distance:int):
    '''Helper method to convert angular (angle distance) coordinates into x and y coordinates'''
    x=distance*sin(angle/180*pi)
    y=distance*cos(angle/180*pi)
    return x,y

def XyToAngular(x:int,y:int):
    '''Helper method to convert x and y coordinates into angle and distance'''
    if x==0 or y==0: return 0, y
    angle=atan(x/y)
    distance=x/sin(angle)
    angle=angle/pi*180
    return angle,distance

def isInsideRect(botLeft,topRight,point):
    '''Checks whether given point (x,y) is inside rectangle defined by point bottom left (x,y) and top right (x,y)'''
    return point[0]>=botLeft[0] and point[0]<=topRight[0] and point[1]>=botLeft[1] and point[1]<=topRight[1]

WARNING_DZ_TOLERANCE=5
'''Lidar deadzone tolerance, robot won't signal an error message if angle is within tolerance of deadzone'''
def readLidar(degree)->int:
    '''Returns the lidar distance at the given angle from the stored array of angles. Also avoids the lidar deadzone'''
    global lidarDOI
    degree=(degree+3600)%360
    if degree>180:
        degree-=360
    if degree<LIDAR_DEADZONE_START-WARNING_DZ_TOLERANCE or degree>LIDAR_DEADZONE_END+WARNING_DZ_TOLERANCE:
        log.warning("lidar danger zone! req. degree: %s, heading: %s" % (degree,getHeading()))
    if degree<LIDAR_DEADZONE_START:
        degree=LIDAR_DEADZONE_START
    if degree>LIDAR_DEADZONE_END:
        degree=LIDAR_DEADZONE_END

    lidarDOI.append(degree)
    return DISTANCE_MAP[(int(degree-90)+360*100)%360]

def readAbsLidar(degree)->int:
    '''Returns the lidar distance at the given absolute angle (not relative to the robot)'''
    return readLidar((degree-getHeading())*-1)

def getAbsX()->int:
    '''Returns the robot distance from the outer wall'''
    if direction==1:
        return readAbsLidar(-90)
    else:
        return 100-readAbsLidar(90)

def getAbsY(back=False)->int:
    '''Returns the robot distance from the back (behind the robot) wall'''
    # return 300-readAbsLidar(0)
    if back: return readLidarBehind()
    else: return 300-readAbsLidar(0)

def findNearestPointAbs(botLeft:tuple,topRight:tuple,back=True,returnRelative:bool=False):
    '''Finds the nearest point inside defined rectangle (bottom left and top right) relative to the bottom left corner of the section'''
    point= findNearestPoint( (botLeft[0]-getAbsX(), botLeft[1]-getAbsY(back)), (topRight[0]-getAbsX(), topRight[1]-getAbsY(back)))
    if point[1]!=-1:
        if returnRelative:
            return point
        else:
            return (point[0]+getAbsX() , point[1]+getAbsY(back))
    else:
        return (-1,-1)
    
def findNearestPoint(botLeft:tuple,topRight:tuple):
    '''Returns the nearest point inside defined rectangle (bottom left and top right) relative to the lidar'''
    global lidarRects
    lidarRects.append((botLeft,topRight))
    topLeftAngular=XyToAngular(botLeft[0],topRight[1])
    botLeftAngular=XyToAngular(botLeft[0],botLeft[1])
    topRightAngular=XyToAngular(topRight[0],topRight[1])
    botRightAngular=XyToAngular(topRight[0],botLeft[1])
    startAngle=min(topLeftAngular[0],botLeftAngular[0])
    endAngle=max(botRightAngular[0],topRightAngular[0])
    nearestPoint=(1000000,10000000)
    for i in range(floor(startAngle),ceil(endAngle)):
        distance=readAbsLidar(i)
        x,y=angularToXy(i,distance)
        if isInsideRect(botLeft,topRight,(x,y)) and y<nearestPoint[1]: nearestPoint=(x,y)
    if nearestPoint[1]==10000000: nearestPoint=(-1,-1)
    return nearestPoint
# LIDAR_TOF_DIST=21
# '''Distance between ultrasonic sensor and lidar'''
# TOFactive=True
# @func_thread()
# def tofLoop():
#     while True:
#         if TOFactive:
#             updTOF()
#         sleep(0.1)
# # def updTOF():
# #     '''Update stored laser time-of-flight sensor reading'''
# #     global lastTOFDist
# #     # t0=time.time()
# #     dist=distSensor.get_distance()/10+LIDAR_TOF_DIST
# #     if dist<200: lastTOFDist=dist
# #     # log.info('dtime: %s'%(time.time()-t0))

def readLidarBehind()->float:
    '''Returns last stored lidar sensor reading at 180° degrees'''
    global lidarDOI
    lidarDOI.append(180)
    return DISTANCE_MAP[90] #behind the robot

def waitLidarBehind(cm,decreasing=True):
    '''Wait until lidar sensor detects distance smaller (or larger) than cm behind the robot'''
    log.info("waitLB cm %s decreasing? %s 0: %s"%(cm,decreasing,readLidarBehind()))
    condition=True
    sleep(0.1)
    while condition:
        dist=readLidarBehind()
        if decreasing: condition=(cm<dist)
        else: condition=(cm>dist)
        sleep(0.05)
    log.info("waitTof out at %s"%readLidarBehind())

def waitAbsLidar(angle:int, cm:int, precision=None, decreasing=True):
    '''Waits until lidar at given angle measures smaller (or larger, based on *decreasing*) distance'''
    condition=True
    precision=None #precision mode is currently disabled
    sleep(0.1)
    lastDist=readAbsLidar(angle)
    
    if (decreasing and readAbsLidar(angle)<=cm) or (not decreasing) and readAbsLidar(angle)>=cm:
        beep_short_parallel()
        log.warning("waitAbsLidar over req. distance! req. dist.: %s, current dist.: %s looking angle %s" % (cm,readAbsLidar(angle),angle))
        return
    while condition:
        distance=readAbsLidar(angle)
        if decreasing:
            condition=distance>=(cm if precision==None else cm+precision)
        else:
            condition=distance<=(cm if precision==None else cm-precision)
        
        if abs(distance-lastDist)>10:
            beep_parallel()
            log.error("waitAbsLidar jump over 10 cm! before: %s , after %s, looking in %s" % (lastDist, distance,angle))
        lastDist=distance
        sleep(0.05)
    if angle!=0: precision=None
    if precision!=None:
        log.debug("distance remaining when switching to waitcm: %s"%readAbsLidar(angle))
        waitCM(readAbsLidar(angle)-cm)
    log.debug("waitAbsLidar over, degree: %s, target cm: %s, actual cm: %s"%(angle,cm,lastDist))
    return lastDist

def waitCM(cm:int):
    '''Waits until robot has traveled given centimeters.'''
    sleep(0.01)
    p0=getMotorPos()
    log.debug("waitcm p0: %s"%p0)
    if cm>0:
        while getMotorPos()<=(p0+cm*TICKS_PER_CM): sleep(0.01)
    else:
        while getMotorPos()>=(p0+cm*TICKS_PER_CM): sleep(0.01)
    log.debug("waitcm finished, pos: %s"%getMotorPos())

def angleDiff(angle1,angle2)->int:
    '''Returns the shortest distance between two angles'''
    diff=((angle1-angle2)+3600)%360
    if diff>180:
        diff=diff-360
    return diff
WAIT_FOR_HEADING_TOLERANCE:int=2
'''The robot will consider itself at the correct angle if actual angle is only off by this much'''
def waitForHeading(tolerance=None, turnDir=0):
    '''Waits until robot faces pilotHeadingTarget (variable) angle
    tolerance: Customizable tolerance, default is the constant WAIT_FOR_HEADING_TOLERANCE (2)
    direction: 1: robot is turning right, -1: robot is turning left, 0: any direction
    '''
    if tolerance==None: tolerance=WAIT_FOR_HEADING_TOLERANCE
    log.debug("waiting for heading %s tolerance: %s turnDir: %s"%(pilotHeadingTarget,tolerance,turnDir))
    if turnDir==0:
        while abs(angleDiff(getHeading(),pilotHeadingTarget))>tolerance:
            sleep(0.01)
    else:
        while angleDiff(getHeading(),pilotHeadingTarget)*turnDir*-1>=tolerance:
            sleep(0.01)
    log.debug("wait for heading done at %s"%getHeading())

def stop(breakForce:int=None, wait:bool=True):
    '''Stops the robot
    breakForce: If given, default breaking (counter-driving) force is overridden
    wait: Whether the program should wait until the robot has stopped before resuming'''
    global actSpeed
    global targetSpeed
    targetSpeed=0
    if breakForce!=None:
        if breakForce==0:
            setVMode(0)
        else:
            setBrakeForce(breakForce)
            setVMode(-2)
    else:
        setVMode(-2)
    if wait:
        while getVMode()==VMODE_BRAKE:
            sleep(0.05)
        sleep(0.3)
    actSpeed=0

def setHeadingTarget(target:int):
    '''Communicates with the ESP what the target angle is'''
    sendCommand(CMD_SET_TARGET_YAW,int((target+heading0)*10))
targetSpeed:int=0
'''Target speed used for raspberry pi side acceleration and deceleration'''
acceleration=5000
'''Acceleration constant, in tick/second^2'''
actSpeed:int=0
'''Actual speed in tick/second, used for raspberry pi side acceleration and deceleration'''

def setTargetSpeed(tSpeed:int):
    '''Sets the raspberry pi side target speed'''
    global targetSpeed
    targetSpeed=tSpeed
    # setSpeed(tSpeed)
lidarRevT0=time.time()

@func_thread()
def pilotLoop():
    '''Loop responsible for logging information and wall following'''
    global wIntegral
    global wLastError
    global actSpeed
    global lidarRevT0
    while True:
        if LidarService.newLidarDataFlag:
            LidarService.newLidarDataFlag=False
            logLidar()
            if time.time()-lidarRevT0>0.3:
                log.error("Lidar took too long! %s"%(time.time()-lidarRevT0))
                beep_twice_parallel()
            lidarRevT0=time.time()
        
    
        
        # print(getHeading())
        # print("d: "+str(readAbsLidar(getHeading())))
        # updBehindCM()
        if pilotMode==PILOT_NONE:
            pass
        elif pilotMode==PILOT_FOLLOW_LEFT:
            pilotDistFromWall=readAbsLidar(pilotHeadingTarget-90)
            # print(pilotDistFromWall)
            error=(pilotDistFromWall-wallTarget)
            wIntegral+=error
            correction=-1*(error*wkP+wIntegral*wkI+(error-wLastError)*wkD)
            # display_data(error)
            if correction<-45: correction=-45
            if correction>20: correction=20
            # print(correction)
            setHeadingTarget(pilotHeadingTarget+correction)
            # sendCommand(CMD_SET_TARGET_YAW,int((pilotDir-correction+yaw0)*10))
            wLastError=error
        sleep(0.01)

lidarLogJSON={"Data":[],"T":-1,"DOI":[],"Rect":[]}
'''Dictionary storing logging data'''
lidarDOI=[]
'''Lidar Degrees Of Interest, degrees that were inspected by readLidar, waitAbsLidar or in any other way'''
lidarRects=[]
'''Rectangles that were checked for the nearest point by findNearestPoint'''
pixyObjects=[]
'''Objects detected by the pixy camera'''

labels = []

values = refr_dict()


def logLidar():
    '''Logging function, logs in a file the lidar distance map, gyro, degrees of interest, checked rectangles, pixy camera objects, camera cutoff lines'''
    global lidarDOI
    global lidarRects
    global pixyObjects
    global DISTANCE_MAP
    if pixyObjects==[]: pixyObjects="Null"
    labels=str(values.items())
    # DISTANCE_MAP[90]=getTOF()
    lidarLogJSON["h0"]=heading0
    lidarLogJSON["heading"]=getHeading()
    lidarLogJSON["Data"]=DISTANCE_MAP
    lidarLogJSON["T"]=time.time()
    lidarLogJSON["DOI"]=lidarDOI
    lidarLogJSON["Rect"]=lidarRects
    lidarLogJSON["cutoffLeft"]=CAM_LEFT_CUTOFF
    lidarLogJSON["cutoffRight"]=CAM_RIGHT_CUTOFF
    lidarLogJSON["cutoffTop"]=CAM_TOP_CUTOFF
    lidarLogJSON["cutoffBot"]=CAM_BOT_CUTOFF
    lidarLogJSON["pixyObjects"]=pixyObjects
    lidarLogJSON["labels"] = labels
    lidarRects=[]
    lidarDOI=[]
    pixyObjects=[]
    t0=time.time()
    dumped=json.dumps(lidarLogJSON)
    if time.time()-t0>0.1:
        log.error("JSON DUMP over %s"%(time.time()-t0))
    t0=time.time()
    log.lidar(dumped)
    if time.time()-t0>0.1:
        log.error("Dump over, elapsed %s"%(time.time()-t0))
        beep_parallel()

def add_label(var_name: str):
    ...

def checkAngle(angle:int)->bool:
    '''Old function used to determine wether there is a wall at given angle'''
    dir=1
    if angle>180: dir=-1
    x=readLidar(90*dir)
    # x=readLidar(90)

    l=readLidar(angle)
    alpha=(90-angle*dir+36000)%360
    correctC=x/cos(alpha/180*pi)
    actualC=readLidar(angle)
    # print(actualC)
    # print(correctC)
    return actualC>correctC+10

def checkSide(side)->int:
    '''Old function used to determine wether there is a wall on a side'''
    corrects=0
    dir=copysign(1,side)
    for i in range(0,80):
        if dir==1:
            if checkAngle(i):
                corrects+=1
        else:
            if checkAngle(360-i):
                # print(360-i)
                corrects+=1
            # print("found at "+str(i))
    return corrects
testVar="TEST"
def openChallengeRun():
    '''The open challenge robot run code'''
    global heading0
    global pilotHeadingTarget
    global direction
    global defaultSpeed
    global direction
    global testVar
    initLoop(open=True)
    values.update("testVar")
    middle = True
    defaultSpeed=defaultSpeed*2
    if readAbsLidar(90) < 15:
        log.info("wall from right")
        direction = 1
        middle = False
    elif readAbsLidar(270) < 15:
        log.info("wall from left")
        direction = -1
        middle = False

    if not middle:
        log.info("targeting middle")
        go(defaultSpeed*2,-25*direction)
        waitAbsLidar(-90*direction,30)
        go(defaultSpeed*2,0)
    else:
        go(defaultSpeed*2,0)
    if middle:
        waitAbsLidar(0,120)
        right=readAbsLidar(90)
        front=readAbsLidar(0)
        left=readAbsLidar(-90)
        rightWall=findNearestPoint((right-10,front-50),(right+10,front-20))
        leftWall=findNearestPoint((-left-10,front-50),(-left+10,front-20))
        if leftWall[0]!=-1:
            direction=DIRECTION_RIGHT
        else:
            direction=DIRECTION_LEFT
    log.info("setting dir to %s"%direction)
    for i in range(12):
        waitAbsLidar(0,55)
        arc(90*direction, defaultSpeed*2,degTolerance=20)
        heading0+=90*direction
        go(defaultSpeed*2,0)
    waitAbsLidar(0,150)
    stop()
    os._exit(1)
lane:float=0
'''Current lane variable'''
LANE_MIDDLE=0
'''Middle lane'''
LANE_LEFT=-1
'''Left lane'''
LANE_RIGHT=1
'''Right lane'''

DIRECTION_RIGHT=1
'''Right direction, the robot turns right at the corners'''
DIRECTION_LEFT=-1
'''Left direction, the robot turns left at the corners'''

parkSide=-1
PARKSIDE_EARLY=0
'''Parking space is at the start of the section'''
PARKSIDE_LATE=1
'''Parking space is at the end of the section'''

ARC_OFFSET_CM_LONG=1
ARC_OFFSET_CM_SHORT=-3 #during a short arc go this much less
SETLANE_SHORT_LIMIT=20
SETLANE_MINIMUM=3
'''How many centimeters the robot gets closer to the wall while turning its steering wheel. Approximation.'''

REAR_AXLE_TO_LIDAR_CM=13
def setLane(wallDirection, targetDistance):
    '''Internal function, moves the robot to targetDistance from the wall
    wallDirection: -1/1 Which wall to move relative to (inner or outer)
    targetDistance: How close to the wall the robot should go'''
    log.info("setlane dir %s tdist %s"%(wallDirection,targetDistance))
    d0=readAbsLidar(90*wallDirection)
    if abs(d0-targetDistance)<SETLANE_MINIMUM:
        go(defaultSpeed,0)
        return
    arcOffset=ARC_OFFSET_CM_LONG
    goAngle=45
    if abs(d0-targetDistance)<=SETLANE_SHORT_LIMIT: 
        arcOffset=ARC_OFFSET_CM_SHORT
        goAngle=20
    if d0<targetDistance:
        go(defaultSpeed,-goAngle*wallDirection)
        waitAbsLidar(90*wallDirection,targetDistance-arcOffset,decreasing=False)
    else:
        go(defaultSpeed,goAngle*wallDirection)
        waitAbsLidar(90*wallDirection,targetDistance+arcOffset,decreasing=True)
    arc(0,defaultSpeed)

def switchLane(newLane:int, insideWall:bool=False):
    '''Switches the lane to the given new lane
    newLane: One of the constants (LANE_LEFT, LANE_RIGHT, LANE_MIDDLE)
    insideWall: True if the robot is moving between two obstacles'''
    global lane
    
    log.info("switching lane to %s from %s ,steep? %s parkpos %s"%(newLane,lane,insideWall,parkPos))
    if newLane==lane:
        log.info("switchlane to same lane")
        go(defaultSpeed,0)
        pass
    else:
        targetDist=23+leftLaneOffset if newLane==LANE_LEFT else 77-rightLaneOffset
        if insideWall:targetDist=100-targetDist
        if direction==DIRECTION_RIGHT:
            setLane(-1 if not insideWall else 1,targetDist)
        else:
            setLane(1 if not insideWall else -1,100-targetDist)

        lane=newLane
            
        log.info("switchlane done (at: %s from opposite wall)"%(readAbsLidar(0)))

def arc(toDegree, speed=None, percent=100, degTolerance=10):
    '''Turns the steering wheel to a percentage then goes until target degree is reached
    toDegree: Target degree
    speed: Speed of the robot, default is the variable defaultSpeed
    percent: How much should the steering wheel turn in percentage. Default is 100'''
    global pilotHeadingTarget
    turnDir=0
    log.debug("arc toDegree %s w speed %s turning percent %s"%(toDegree,speed,percent))
    if speed==None: speed=defaultSpeed
    setSteerMode(SMODE_NONE)
    if toDegree>getHeading():
        steer(copysign(percent,speed))
        turnDir=1
    else:
        steer(-copysign(percent,speed))
        turnDir=-1
    setTargetSpeed(speed)
    
    pilotHeadingTarget=toDegree
    setHeadingTarget(toDegree)
    #
    
    setVMode(int(copysign(1,speed)))
    
    waitForHeading(tolerance=degTolerance,turnDir=turnDir)
    setSteerMode(SMODE_GYRO)
    waitForHeading(tolerance=3,turnDir=turnDir)
    log.info("arc over at %s"%getHeading())

def turnCorner(goForward=False):
    '''Turns the robot in the corner'''
    global dontReverse
    global heading0
    global lane
    side=lane*direction
    log.info("turncorner, goforward %s"%goForward)
    if side==LANE_LEFT: # outer
        waitAbsLidar(0,55)
        if goForward:
            arc(90*direction)
        if not goForward:
            arc(75*direction)
            stop()
            go(-defaultSpeed,90*direction)
            waitLidarBehind(50)
            stop()

        lane=-0.5*direction
    else: #inner
        if goForward:
            waitAbsLidar(0,55)
            arc(90*direction)
        else:
            waitAbsLidar(0,20)
            stop()
            arc(90*direction,-defaultSpeed)
            waitLidarBehind(50)
            stop()
        lane=0.5*direction
    heading0+=90*direction
    log.info("turncorner over")
    dontReverse = False
lastHeadingLock=threading.Lock()
def getHeading():
    '''Returns last heading value'''
    return (getRawHeading()/10)-heading0

#This function gets called 100 times a second
def accelerate():
    '''Accelerates'''
    global actSpeed
    # # updHeading()
    
    if actSpeed!=targetSpeed:
        if actSpeed>targetSpeed: actSpeed=(max(actSpeed-acceleration/20,targetSpeed))
        if actSpeed<targetSpeed: actSpeed=(min(actSpeed+acceleration/20,targetSpeed))
        log.info("setting speed %s"%actSpeed)
        setSpeed(actSpeed)
    # pass
SerialCommunicationService.accelFunc=accelerate
RUN: bool = False
'''Used for starting the robot run'''

@func_thread()
def checkForInputLoop():
    '''Function to detect input and act accordingly'''
    global RUN
    global outLoop
    while True:
        inputted = input()
        match inputted:
            case "run":
                RUN = True
            case "exit":
                displayString("EXIT")
                GPIO.cleanup()
                log.critical("__EXIT__")
                os._exit(1)
            case "test":
                if not outLoop:
                    outLoop = True
                else:
                    outLoop = False
                
            case text if text.startswith("exec "):
                try:
                    exec(text[5:])
                except Exception as e: log.error(e)

def checkAngleFromWall()->int:
    '''Calculates the robots angle relative to the wall'''
    a=readLidar(90)
    c=readLidar(60)
    beta=30/180*pi
    #cosine theorem: a^2=b^2+c^2-2*b*c*cos(alpha)
    b=sqrt(a**2+c**2-2*a*c*cos(beta))
    #law of sines: a/sin(alpha)=b/sin(beta)
    alpha=asin(sin(beta)*a/b)
    gamma=pi-alpha-beta
    return gamma*180/pi

def checkLidarDeadzone()->tuple[int,int]:
    '''Checks the lidar deadzone's limits'''
    leftLimit=0
    rightLimit=0
    for i in range(180):
        # log.info("%s %s %s"%(i,DISTANCE_MAP[(i-90+3600)%360],DISTANCE_MAP[(-i-90+3600)%360]))
        if DISTANCE_MAP[(i-90+3600)%360]<20 and rightLimit==0:
            rightLimit=-i
        if DISTANCE_MAP[(-i-90+3600)%360]<20 and leftLimit==0:
            leftLimit=i
    return (leftLimit,rightLimit)



#Startup display mode constants
DISPLAY_LIDAR=0
DISPLAY_LK=1
DISPLAY_ENC0=2
DISPLAY_ENC1=3
DISPLAY_IMU=4
DISPLAY_BH=5
DISPLAY_LIDAR_DZ=6
DISPLAY_ANGLE_FROM_WALL=7

#Diameter of 70
SERVO_MAX=435
SERVO_MIN=228
SERVO_SAFE_MAX=SERVO_MAX-1
SERVO_SAFE_MIN=SERVO_MIN+9

#ARC RIGHT and ARC LEFT diameter (rear axle center-rear axle center)=45 cm

EMERGENCY_START_PIN=4



def initLoop(open=False):
    '''Displays debug information and waits until a button is pressed
    open: Set to true to display open challenge relevant information'''
    global heading0
    global direction
    global outLoop
    global parkPos
    global parkSide
    pixy.init()
    beep()
    dispMode=DISPLAY_LIDAR
    displayData(10101010)
    log.info("10101010")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EMERGENCY_START_PIN,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    
    # sync()
    checkForInputLoop()
    # distSensor.open()
    # distSensor.start_ranging(VL53L1xDistanceMode.SHORT)
    
    # sleep(5)

    outLoop = False
    
    SerialCommunicationService.startReading()

    getRawHeading()
    setServoMin(SERVO_SAFE_MIN) #SERVO SAFE LIMIT
    setServoMax(SERVO_SAFE_MAX) #SERVO SAFE LIMIT
    log.info("SET")
    sleep(0.2)
    heading0=getRawHeading()/10
    setBrakeForce(20)
    pilotLoop()
    
    last_qyro = getHeading()
    pressed:bool=False
    while not pressed:
        sleep(0.5)
        log.debug("parkpos %s"%parkPos)
        parkPos=-1
        if outLoop:
            while True:
                sleep(0.1)
                if not outLoop:
                    break

        heading0=getRawHeading()/10
        if TM.switches[0] or RUN or (GPIO.input(EMERGENCY_START_PIN)==GPIO.LOW):
            setLeds("11111111")
            sleep(1)
            pressed=True
        setLed(0, last_qyro != heading0)
        # log.info("last us %s"%getRawUS())
        last_qyro = heading0
        objPos:tuple=(-2,-2)
        color:int=None
        if getAbsY() > 160:
            objPos=(-1,-1)
        else:
            objPos=[-1,-1]
            if objPos[1] > -1:
                # color=colorMatrixPrequel[0]
                if not open:
                    color=checkColor()
                    # color=GREEN
                else: color=RED
                if color == GREEN:
                    setLed(3,True)
                else:
                    setLed(3, False)
                if color == RED:
                    setLed(2,True)
                else:
                    setLed(2,False)
            else:
                setLed(3,False)
                setLed(2,False)
        if dispMode==DISPLAY_LIDAR: #display lidar distance at angle 0
            displayData("L.",readLidar(0))
        elif dispMode==DISPLAY_ENC0: #display left motor encoder
            displayData("E0.",getLPos())
        elif dispMode==DISPLAY_ENC1: #display right motor encoder
            displayData("E1.",getRPos())
        elif dispMode==DISPLAY_IMU: #display gyroscope
            displayData("I.",getRawHeading())
        elif dispMode==DISPLAY_LK: #turns on every led to check led&key
            displayString("88888888")
            setLeds("11111111")
        elif dispMode==DISPLAY_BH: #displays laser distance sensor
            displayData("B.",readLidarBehind())
            # displayData("B.",-1)
        elif dispMode==DISPLAY_LIDAR_DZ: #displays lidar deadzone limits
            limits=checkLidarDeadzone()
            displayString("D.%s  %s"%(limits[0],limits[1]))
        elif dispMode==DISPLAY_ANGLE_FROM_WALL:
            displayData("A.",checkAngleFromWall())
        if TM.switches[1]: #switch display mode
            displayString("--------")
            sleep(0.2)
            dispMode=(dispMode+1)%8
        if TM.switches[7]:
            displayString("EXIT")
            log.critical("__EXIT__")
            GPIO.cleanup()
            os._exit(1)
        front=readAbsLidar(0)
        left=readAbsLidar(-90)
        right=readAbsLidar(90)
        parkingPlaceRight=findNearestPoint((right-20,front-105),(right-10,front-95))
        parkingPlaceLeft=findNearestPoint((-left+10,front-105),(-left+20,front-95))
        rightWall=findNearestPoint((right-10,front-90),(right+10,front-70))
        leftWall=findNearestPoint((-left-10,front-90),(-left+10,front-70))
        # if open:
            
        # else:
        #     rightWall=findNearestPointAbs((90,205),(110,232),back=False)
        #     leftWall=findNearestPointAbs((-10,205),(10,232),back=False)
        if rightWall[0]==-1:

            setLed(7,True)
        else:
            setLed(7,False)
        
        if leftWall[0]==-1:
            direction=DIRECTION_LEFT
            setLed(6,True)
        else:
            direction=DIRECTION_RIGHT
            setLed(6,False)
        if parkingPlaceLeft[0]!=-1:
            log.debug("parking lot found left, !shouldnt happen")
            direction=DIRECTION_RIGHT
            parkPos=0
            parkSide=PARKSIDE_LATE
            setLed(4,True)
        else:
            setLed(4,False)
        if parkingPlaceRight[0]!=-1:
            log.debug("parking lot found right")
            direction=DIRECTION_LEFT
            parkPos=0
            parkSide=PARKSIDE_LATE
            setLed(5,True)
        else:
            setLed(5,False)
    log.info("Initloop over")
    return color, objPos

trafficSignMatrix = [
    [0,0,0],
    [0,0,0],
    [0,0,0],
    [0,0,0]
]
'''For storing the detected traffic signs'''

section = 0
'''Current section'''

def findFirst(section):
    '''Returns the color of the first traffic sign in the section'''
    colorPos=0
    log.info("ff section: %s"%trafficSignMatrix[section])
    for i in range(3):
        if trafficSignMatrix[section][i]!=0:
            log.info("findfirst %s i %s"%(trafficSignMatrix[section][i],i))
            return trafficSignMatrix[section][i],i

def findLast(section):
    '''Returns the color of the last traffic sign in the section'''
    colorPos=0
    for i in range(3):
        if trafficSignMatrix[section][2-i]!=0: return trafficSignMatrix[section][2-i],i

def findLastInLap():
    log.info("find last in lap, matrix: %s"%trafficSignMatrix)
    if trafficSignMatrix[0][0]!=0 or trafficSignMatrix[0][1]!=0:
        return findFirst(0)
    else:
        return findLast(3)
dontReverse = False
'''Used for optimization'''
def turnAround():
    '''180° turn around a red traffic sign'''
    # global dontReverse
    global heading0
    global doDetection
    global lane
    doDetection=False
    if lane==LANE_RIGHT: #always the case
        arc(-90,speed=safeSpeed)
        idealDistance=40+leftLaneOffset
        if readAbsLidar(-90)>idealDistance:
            waitAbsLidar(-90,idealDistance)
        else:
            stop()
            go(-safeSpeed,-90)
            waitAbsLidar(-90,idealDistance+5,decreasing=False)
            stop()
            go(safeSpeed,-90)
            waitAbsLidar(-90,idealDistance)
        arc(-180,speed=safeSpeed)
        heading0-=180

parkPos=-1
'''The position of the parking space'''
doDetection=True
'''Set to false if all traffic signs are detected'''
defaultSpeed=2500
'''The default speed, most functions use this by default'''
safeSpeed=2000
'''Slower speed'''
def reverseMatrix():
    '''Reverses the trafficSignMatrix when turning around. Also sets the parking space position accordingly'''
    global trafficSignMatrix
    global parkPos, parkSide
    parkPos=(4-parkPos)%4
    parkSide=1-parkSide
    trafficSignMatrix[1],trafficSignMatrix[3]=trafficSignMatrix[3],trafficSignMatrix[1]
    for i in range(len(trafficSignMatrix)):
        trafficSignMatrix[i].reverse()
        for j in range(len(trafficSignMatrix[i])):
            trafficSignMatrix[i][j]*=-1
rightLaneOffset, leftLaneOffset=0,0
'''Lane offsets for when there is a parking space next to the robot'''
startTime=-1
'''T0, when the robot starts'''
finalSection=12
'''Final section variable, if the robot turns around is decreased by 1'''

def checkObjectColor(absObjPos:tuple,back=True,useLED=True, goBack=50)->int:
    global lane
    
    ox,oy=absObjPos[0],absObjPos[1]
    ox-=getAbsX()
    oy-=getAbsY(back=back)
    oy+=6
    distance=sqrt(ox**2+oy**2)
    angle=atan(ox/oy)*180/pi
    log.info("obj found %s dist %s angle %s rel x %s y %s"%(absObjPos,distance,angle,ox,oy))
    go(defaultSpeed,angle)
    pixy.set_lamp(1,1)
    waitCM(distance-25)
    stop()
    lane=0
    signColor = checkColor()
    log.debug("COC signcolor=%s"%signColor)
    pixy.set_lamp(0,0)
    go(-defaultSpeed,0)

    waitCM(-goBack)
    stop()
    return signColor
def isInParkingSection()->bool:
    return section%4==parkPos%4 and parkPos!=-1
def obstacleChallengeRun():
    '''The obstacle challenge robot run'''
    global direction
    global startTime
    global finalSection
    global trafficSignMatrix
    global section
    global doDetection
    global dontReverse
    global leftLaneOffset
    global rightLaneOffset
    global parkPos
    global lane
    global parkSide
    global heading0
    log.info("START")
    color, signObject = initLoop(open=False)
    log.debug("after initloop over parkpos: %s"%parkPos)
    signObject=findNearestPointAbs((30,175),(70,225),back=False)
    log.info("IO")
    startTime=time.time()
    log.info("Direction is %s"%direction)
    
    if parkPos==0:
        if direction==DIRECTION_LEFT:
            log.debug("set offset (DL)")
            rightLaneOffset=20
            leftLaneOffset=0
        else:
            log.debug("set offset (DR)")
            leftLaneOffset=20
            rightLaneOffset=0
    if signObject[1]==-1:
        switchLane(LANE_LEFT*direction,insideWall=isInParkingSection())
    else:
        color=checkObjectColor(signObject,back=False)
        switchLane(LANE_LEFT if color==GREEN else LANE_RIGHT,insideWall=isInParkingSection())
        trafficSignMatrix[section%4][2]=color*(-1 if signObject[0] < 50 else 1)
    turnCorner()

    i=1
    while i<finalSection:
        log.debug("parkside: %s"%parkSide)
    # for i in range(1,13):
        section=i
        log.section = section
        if parkPos==-1:
            if direction==DIRECTION_LEFT:
                parkObj=findNearestPointAbs((80,90),(90,210))
            else:
                parkObj=findNearestPointAbs((10,90),(20,210))
            px,py=parkObj[0],parkObj[1]
            if py!=-1:
                parkPos=section%4
                if py>150:
                    parkSide=PARKSIDE_LATE
                else:
                    parkSide=PARKSIDE_EARLY

        if parkPos%4==section%4 and parkPos!=-1:
            if direction==DIRECTION_RIGHT:
                leftLaneOffset=20
                rightLaneOffset=0
            else:
                rightLaneOffset=20
                leftLaneOffset=0
        else:
            leftLaneOffset=0
            rightLaneOffset=0

        if doDetection:
            #DETECTION

            
            signRow = -1
            signColumn = 0
            signColor = -1
            signPos = findNearestPointAbs((30,80),(70,170),back=True)
            sx, sy = signPos[0], signPos[1]
            
            if sy != -1:
                signRow = 0 if sy < 125 else 1
                signColumn = -1 if sx < 50 else 1

                signColor=checkObjectColor(signPos, goBack=85 if isInParkingSection() and parkSide==PARKSIDE_EARLY and signRow==1 else 50) #go back more if the traffic sign is in the middle, we are in the parking section and the parking place is close to the start of the section, so the parking space walls do not interfere.
                
                
                # waitTOF(50)
                trafficSignMatrix[section%4][signRow] = signColor * signColumn
            if section== 4:
                doDetection = False
        
        go(defaultSpeed,0)
        if doDetection:
            #STILL DETECT
            
            if signRow != -1:
                switchLane(LANE_LEFT if signColor == GREEN else LANE_RIGHT)
            else:
                switchLane(-direction)
            
            if signRow != 1:
                # waitTOF(120,decreasing=False)
                waitLidarBehind(120,decreasing=False)
                signPos = findNearestPointAbs((25,180),(75,220),back=False)
                sx, sy = signPos[0], signPos[1]
                if sy != -1:
                    signRow = 2
                    signColumn = -1 if sx < 50 else 1

                    signColor=checkObjectColor(signPos,back=False)
                    
                    # stop()
                    trafficSignMatrix[section%4][signRow] = signColor * signColumn
                if signRow == 2:
                    switchLane(LANE_LEFT if signColor == GREEN else LANE_RIGHT, insideWall=isInParkingSection())
                else:
                    switchLane(-direction,insideWall=isInParkingSection())
                
            if signRow == -1: log.error("Empty section!!!!!")

        else:
            #OPTIMIZED - NO DETECTION
            if section==8 and abs(findLastInLap()[0])==RED:
                switchLane(LANE_RIGHT)
                if trafficSignMatrix[0][1]!=0:
                    waitAbsLidar(0,140)
                else:
                    waitAbsLidar(0,190)
                log.info("TURNING AROUND")
                turnAround()
                reverseMatrix()
                direction*=-1
            else:
                switchLane(LANE_LEFT if abs(findFirst(section%4)[0]) == GREEN else LANE_RIGHT)
                
                if abs(trafficSignMatrix[section%4][2])!=abs(findFirst(section%4)[0]) and trafficSignMatrix[section%4][0]*trafficSignMatrix[section%4][2]!=0: #there is a traffic sign on the first and last row and they are of different color
                    waitLidarBehind(120,decreasing=False)
                    switchLane(LANE_LEFT if abs(trafficSignMatrix[section%4][2]) == GREEN else LANE_RIGHT, insideWall=isInParkingSection())
                if trafficSignMatrix[section%4][1]==0 and trafficSignMatrix[section%4][2]==0: #no traffic sing on middle or last row
                    log.info("no row 1-2, going to outer")
                    waitLidarBehind(120,decreasing=False)
                    switchLane(-direction,insideWall=isInParkingSection())
        #JOINED
        goForward=False
        if not doDetection:
            if lane==-direction: #outer
                if findFirst((section+1)%4)[1]!=0: #first of next section is not on 0th row
                    if not(section+1==8 and abs(findLastInLap()[0])==RED): #wont be turning around next section
                        if (section+1)%4!=parkPos%4: #next section doesn't have the parking space (would cause too much trouble)
                            if section!=finalSection-1: # not turning corner to the final section
                                goForward=True
        turnCorner(goForward)
        i+=1
    #FINISHED 3 LAPS, RIGHT BEFORE STARTING SECTION, PARKING
    section+=1
    if parkPos%4==section%4 and parkPos!=-1:
        if direction==DIRECTION_RIGHT:
            leftLaneOffset=20
            rightLaneOffset=0
        else:
            rightLaneOffset=20
            leftLaneOffset=0
    else:
            leftLaneOffset=0
            rightLaneOffset=0
    
    if ((abs(trafficSignMatrix[0][0])==RED and direction==DIRECTION_RIGHT) or (abs(trafficSignMatrix[0][0])==GREEN and direction==DIRECTION_LEFT) ): #do we need to go to the inner wall
        switchLane(direction) #inner
    else:
        switchLane(-direction) #outer
    waitLidarBehind(110,decreasing=False)
    stop()
    sleep(6)

    switchLane(-direction) #outer
    
    for i in range(parkPos):
        go(defaultSpeed,0)
        if (i+1)==parkPos:
            waitAbsLidar(0,42+20)
        else:
            waitAbsLidar(0,42)
        arc(90*direction)
        heading0+=90*direction
    if direction==DIRECTION_RIGHT: #set correct offsets
        leftLaneOffset=20
        rightLaneOffset=0
    else:
        rightLaneOffset=20
        leftLaneOffset=0
    switchLane(-direction) #outer
    if parkSide==PARKSIDE_EARLY:
        idealPosition=134 #ideal position to arc from to park
    else:
        idealPosition=70 #ideal position to arc from to park
    waitAbsLidar(0,idealPosition-10)
    stop()
    go(-safeSpeed,0)
    waitAbsLidar(0,idealPosition,decreasing=False)
    if direction==DIRECTION_RIGHT:
        arc(-90,-safeSpeed)
    else:
        arc(90,-safeSpeed)
    stop()
    heading0-=90*direction
    parked=False
    while not parked:
        leftPWall=findNearestPoint((-30,10),(0,60))
        rightPWall=findNearestPoint((0,10),(30,60))
        axleOffset=sin(getHeading()/180*pi)*REAR_AXLE_TO_LIDAR_CM
        go(safeSpeed,(leftPWall[0]+rightPWall[0]+axleOffset)*2)
        if (leftPWall[1]+rightPWall[1])/2<20: parked=True
        sleep(0.05)
    goUnreg(15,0)
    sleep(3)

    stop()
    #FINISH
    log.error("FINISHED WITH TIME %s"%(time.time()-startTime))

            
    
ARC_OFFSET_CM_20=6
def testRun():
    '''Test run used for testing'''
    global pilotHeadingTarget
    global trafficSignMatrix
    global direction
    global heading0
    global parkPos, parkSide, section
    global leftLaneOffset, rightLaneOffset
    initLoop(open=False)
    arc(179,defaultSpeed)
    stop()
obstacleChallengeRun() #We start the obstacle challenge run
stop()
#END

os._exit(1)