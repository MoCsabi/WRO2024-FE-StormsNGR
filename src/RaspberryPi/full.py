#! /usr/bin/python
'''The main python file, it contains the code for both robot runs and helper functions'''
from ctypes import Structure, c_uint
import json
from math import cos, pi, atan, ceil, copysign, floor, sin
import threading
from time import sleep, time
from log import *
from Reader import I2CReader
from TMBoard import *
from ESP32CommunicationService import *
import LidarService
from LidarService import *
from Buzzer import *


TICKS_PER_CM=31.43
'''Convert internal ticks to CM. (Motor rotation is measured in ticks by the ESP)'''

direction:int=1
'''Stores the detected randomized direction during obstacle challange. 
1: robot turns right
-1: robot turns left'''

GREEN=1
'''Constant for the color Green. Used for obstacle color detection and storing.'''
RED=2
'''Constant for the color Red. Used for obstacle color detection and storing.'''
# import pixy
#TODO: Pixy camera curretnly not in use
# pixy.init()
colorMatrixPrequel = [GREEN,RED]
getColorCounter = 1
#currently camera is not in use
def checkColor(attempts:int=5)->int:
    '''Checks the color of the closest obstacle using the Pixy camera. 
    attempts: Not recommended to have over 1. Number of attempts with a delay of 0.1 seconds between if first attempt is not succesfull.'''
    global getColorCounter
    i=0
    rt=-1
    
    getColorCounter += 1
    log.info("color: %s"%colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)])
    return colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)]
    while rt==-1 and i<attempts:
        blocks=pixy.BlockArray(4)
        n=pixy.ccc_get_blocks(4,blocks)
        logLevel("detected %s blocks"%n,INFO)
        for i in range(n):
            b:pixy.Block=blocks[i]
            log.debug("detected at %s %s, size %s %s with signature: %s"%(b.m_x,b.m_y,b.m_width,b.m_height,b.m_signature))
        if n>1:
            log.warning("multiple blocks detected!")
        if n==0:
            beep_parallel()
            log.warning("no blocks detected!")
            rt=-1
        else:            
            b1:pixy.Block=blocks[0]
            rt= GREEN if b1.m_signature==1 else RED

        i+=1
        sleep(0.1)
    if rt==-1:
        log.error("did not find object!")
        rt=GREEN
    log.info("Detected final color %s"%("GREEN" if rt==GREEN else "RED"))
    return rt

wIntergral=0
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
'''Wall distance following target distance in centimetres'''
pilotHeadingTarget:int=0
'''Target robot direction'''
lastHeading=0
'''Last heading direction recorded'''
heading0=0
'''Robot starting direction'''

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

VMODE_FORWARD=1
'''vMode (velocity), sets it so the robot may move forward'''
VMODE_BACKWARD=-1
'''vMode (velocity), sets it so the robot may move backward'''
VMODE_STOP=0
'''vMode (velocity), sets it so the robot stops (no drive on wheels)'''
VMODE_BRAKE=-2
'''vMode (velocity), sets it so the robot stops by brakeing (counter-drive on wheels)'''

SMODE_NONE=0
'''sMode (steer), sets it so the robot does not steer'''
SMODE_GYRO=1
'''sMode (steer), sets it so the robots steering is kept straight by the gyro'''

def go(speed:int,headingTarget:int, pilotModeIn:int=PILOT_NONE, wallDistance:int=-1):
    '''Starts the robot with given parameters
    speed: speed of the robot measured in ticks/second. (0-3000)
    headingTarget: Target direction in degrees (-90 - 90)
    pilotMode: If left at default value no piloting, otherwise set given pilotmode (PILOT_FOLLOW_LEFT, PILOT_FOLLOW_RIGHT)
    wallDistance: Only relevant if pilotMode is not PILOT_NONE, sets target wall distance for wall following'''
    global pilotHeadingTarget
    global wIntergral
    log.debug("go with speed: %s headingT: %s pilotMode: %s walldist: %s"%(speed,headingTarget,pilotModeIn,wallDistance))
    #reset wall following integral variable to ansure no previous buildup is kept
    wIntergral=0
    setTargetSpeed(speed)
    setPilotMode(pilotModeIn,wallDistance)
    pilotHeadingTarget=headingTarget
    setHeadingTarget(headingTarget)
    #
    setSteerMode(1)
    setVMode(int(copysign(1,speed)))

def angularToXy(angle:int, distance:int):
    x=distance*sin(angle/180*pi)
    y=distance*cos(angle/180*pi)
    return x,y
def XyToAngular(x:int,y:int):
    if x==0 or y==0: return 0, y
    angle=atan(x/y)
    distance=x/sin(angle)
    angle=angle/pi*180
    return angle,distance
def isInsideRect(botLeft,topRight,point):
    return point[0]>=botLeft[0] and point[0]<=topRight[0] and point[1]>=botLeft[1] and point[1]<=topRight[1]
def  readLidar(degree)->int:
    global lidarDOI
    if (degree+360*100)%360>LIDAR_DANGERZONE_START and (degree+360*100)%360<LIDAR_DANGERZONE_END:
        beep_parallel()
        log.error("lidar danger zone! req. degree: %s, heading: %s" % (degree,getHeading()))
    if degree not in lidarDOI: lidarDOI.append(degree)
    return TERKEP_DISTANCE[(int(degree-90)+360*100)%360]

def readAbsLidar(degree)->int:
    return readLidar((degree-getHeading())*-1)

def getAbsX()->int:
    if direction==1:
        return readAbsLidar(-90)
    else:
        return 100-readAbsLidar(90)
def getAbsY()->int:
    return 300-readAbsLidar(0)

def findNearestPointAbs(botLeft:tuple,topRight:tuple):
    point= findNearestPoint( (botLeft[0]-getAbsX(), botLeft[1]-getAbsY()), (topRight[0]-getAbsX(), topRight[1]-getAbsY()))
    if point[1]!=-1:
        return (point[0]+getAbsX() , point[1]+getAbsY())
    else:
        return (-1,-1)
def findNearestPoint(botLeft:tuple,topRight:tuple):
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
def waitAbsLidar(angle:int, cm:int, precision=None, decreasing=True):
    condition=True
    precision=None #precision mode is currently disabled
    lastDist=readAbsLidar(angle)
    if (decreasing and readAbsLidar(angle)<=cm) or (not decreasing) and readAbsLidar(angle)>=cm:
        beep_parallel()
        log.warning("waitAbsLidar already over req. distance! req. dist.: %s, current dist.: %s looking in angle %s" % (cm,readAbsLidar(angle),angle))
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
def waitCM(cm:int):
    sleep(0.01)
    p0=getAvgPos()
    log.debug("waitcm p0: %s"%p0)
    if cm>0:
        while getAvgPos()<=(p0+cm*TICKS_PER_CM): sleep(0.01)
    else:
        while getAvgPos()>=(p0+cm*TICKS_PER_CM): sleep(0.01)
    log.debug("waitcm finished, pos: %s"%getAvgPos())
WAIT_FOR_HEADING_TOLERANCE:int=2
def waitForHeading():
    while abs(getHeading()-pilotHeadingTarget)>WAIT_FOR_HEADING_TOLERANCE: sleep(0.01)

def stop(breakForce:int=None, wait:bool=True):
    global actSpeed
    global targetSpeed
    targetSpeed=0
    if breakForce!=None:
        if breakForce==0:
            setVMode(0)
        else:
            setBreakForce(breakForce)
            setVMode(-2)
    else:
        setVMode(-2)
    if wait:
        while abs(getSpeed())>=100:
            sleep(0.01)
    actSpeed=0

def setHeadingTarget(target:int):
    sendCommand(CMD_SET_TARGET_YAW,int((target+heading0)*10))
targetSpeed:int=0
acceleration=5000
actSpeed:int=0
def setTargetSpeed(tSpeed:int):
    global targetSpeed
    targetSpeed=tSpeed


def pilotRobot():
    global wIntergral
    global wLastError
    global actSpeed
    logLidar()
    updHeading()
    # print(getHeading())
    # print("d: "+str(readAbsLidar(getHeading())))
    if actSpeed!=targetSpeed and (getVMode()==1 or getVMode()==-1):
        if actSpeed>targetSpeed: actSpeed=(max(actSpeed-acceleration/10,targetSpeed))
        if actSpeed<targetSpeed: actSpeed=(min(actSpeed+acceleration/10,targetSpeed))
        log.info("setting speed %s"%actSpeed)
        setSpeed(actSpeed)
    if pilotMode==PILOT_NONE:
        pass
    elif pilotMode==PILOT_FOLLOW_LEFT:
        pilotDistFromWall=readAbsLidar(pilotHeadingTarget-90)
        # print(pilotDistFromWall)
        error=(pilotDistFromWall-wallTarget)
        wIntergral+=error
        correction=-1*(error*wkP+wIntergral*wkI+(error-wLastError)*wkD)
        # display_data(error)
        if correction<-45: correction=-45
        if correction>20: correction=20
        # print(correction)
        setHeadingTarget(pilotHeadingTarget+correction)
        # sendCommand(CMD_SET_TARGET_YAW,int((pilotDir-correction+yaw0)*10))
        wLastError=error
lidarLogJSON={"Data":[],"T":-1,"DOI":[],"Rect":[]}
lidarDOI=[]
lidarRects=[]
def logLidar():
    global lidarDOI
    global lidarRects
    lidarLogJSON["Data"]=TERKEP_DISTANCE
    lidarLogJSON["T"]=time()
    lidarLogJSON["DOI"]=lidarDOI
    lidarLogJSON["Rect"]=lidarRects
    lidarRects=[]
    lidarDOI=[]
    log.lidar(json.dumps(lidarLogJSON))
def checkAngle(angle:int)->bool:
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
def openChallangeRun():
    global heading0
    global pilotHeadingTarget
    initLoop()
    dir = 0
    middle = True
    go(3000,0)
    waitAbsLidar(0,100)
    stop()
    go(-3000,0)
    waitCM(-100)
    stop()
    sleep(20000)
    if readAbsLidar(90) < 15:
        log.info("wall from right")
        dir = 1
        middle = False
    elif readAbsLidar(270) < 15:
        log.info("wall from left")
        dir = -1
        middle = False

    if not middle:
        log.info("targeting middle")
        go(2000,-25*dir)
        waitAbsLidar(-90*dir,30)
        go(2000,0)
    else:
        if checkSide(0) > 2: dir = -1
        else: dir = 1
        go(2000,0)
    log.info("setting dir to %s"%dir)
    for i in range(12):


        waitAbsLidar(0,70)
        display_data(readAbsLidar(0))
        setPilotMode(PILOT_NONE)
        setSpeed(2000)
        setHeadingTarget(90*dir)

        pilotHeadingTarget=90*dir
        waitForHeading()
        heading0+=90*dir
        updHeading()
        go(3000,0)
        waitAbsLidar(0,150)
        wD=readAbsLidar(-90*dir)
        go(3000,0,PILOT_FOLLOW_LEFT if dir == 1 else PILOT_FOLLOW_RIGHT,wD)

    stop()
    exit()
lane:float=0
LANE_MIDDLE=0
LANE_LEFT=-1
LANE_RIGHT=1
def switchLane(newLane:int, steep:bool=False):
    global lane
    log.info("switching lane to %s from %s ,steep? %s"%(newLane,lane,steep))
    if newLane==lane:
        pass
    else:
        if newLane>lane:
            dir=1
        else:
            dir=-1
        lane=newLane
        if steep:
            go(1000,50*dir)
        else:
            go(1000,40*dir)
        
        log.info("switchlane start turn (at: %s from opposite wall)"%readAbsLidar(0))
        if steep:
            waitAbsLidar(90*dir,25,decreasing=True)
        else:
            waitAbsLidar(-90*direction,25 if dir==-1*direction else 75,decreasing=(dir==-1*direction))

        go(1000,0)
        log.info("switchlane at correct walldist (at: %s from opposite wall, walldist(-90): %s)"%(readAbsLidar(0),readAbsLidar(-90)))
        waitForHeading()
        log.info("switchlane heading is correct (at %s from opposite wall)"%readAbsLidar(0))
def turnCorner():
    global heading0
    global lane
    side=lane*direction
    if side==LANE_LEFT: # outer
        waitAbsLidar(0,64)
        go(1000,90*direction)
        waitForHeading()
        stop()
        log.info("stopped at turncorner")
        heading0+=90*direction
        updHeading()
        lane=-0.5
    else: #inner
        waitAbsLidar(0,17)
        heading0+=90*direction
        updHeading()
        go(-1000,0)
        waitForHeading()
        lane=0.5
    return
def updHeading():
    global heading0
    global lastHeading
    lastHeading=(requestData(CMD_DATA_GYRO)/10-heading0)
def getHeading()->int:
    return lastHeading
def initLoop():
    global heading0
    global direction
    display_data(10101010)
    sync()
    # sleep(5)
    heading0=requestData(CMD_DATA_GYRO)/10
    LidarService.onLidarRev=pilotRobot
    startLidarService()

    last_qyro = getHeading()

    sleep(0.2)
    
    pressed:bool=False
    while not pressed:
        sleep(0.5)
        if TM.switches[0]:
            set_leds("11111111")
            sleep(1)
            pressed=True
        setLed(0, last_qyro != getHeading())
        last_qyro = getHeading()
        object:tuple=(-2,-2)
        color:int=None
        if getAbsY() > 160:
            setLed(5,True)
        else:
            object=findNearestPointAbs((30,175),(70,225))
            if object[1] > -1:
                setLed(5, False)
                color = colorMatrixPrequel[0]
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
                setLed(5, True)
        display_data(readAbsLidar(0))
        log.info("x: %s y: %s"%(getAbsX(),getAbsY()))
        if findNearestPointAbs((90,205),(110,232))[1]==-1:

            setLed(7,True)
        else:
            setLed(7,False)
        if findNearestPointAbs((-10,205),(10,232))[1]==-1:
            direction=-1
            setLed(6,True)
        else:
            direction=1
            setLed(6,False)
    
    return color, object

parkPos = -1
trafficSignMatrix = [
    [0,0,0],
    [0,0,0],
    [0,0,0],
    [0,0,0]
]

currentSection = -1

def checkForParkingArea():
    pass

def obstacleChallangeRun():
    color, object = initLoop()
    log.info("Direction is %s"%direction)
    global trafficSignMatrix
    global currentSection
    # TODO: making it so that it actually works

    if color == GREEN:
        switchLane(LANE_LEFT,False)
    elif color == RED:
        switchLane(LANE_RIGHT,False)
    turnCorner()
    for i in range(15):
        currentSection = i
        go(-1000,0)
        waitAbsLidar(0,255,decreasing=False)
        stop()
        if parkPos == -1:
            checkForParkingArea()
        signRow = -1
        signColumn = 0
        signColor = -1
        signPos = findNearestPointAbs((30,80),(70,170))
        sx, sy = signPos[0], signPos[1]

        if sy != -1: 
            signRow = 0 if sy < 125 else 1
            signColumn = -1 if sx < 50 else 1
            signColor = checkColor()
            trafficSignMatrix[currentSection%4][signRow] = signColor * signColumn
        go(1000,0)
        if signRow != -1:
            switchLane(LANE_LEFT if signColor == GREEN else LANE_RIGHT)
        if signRow != 1:
            waitAbsLidar(0,180)
            signPos = findNearestPointAbs((25,180),(75,220))
            sx, sy = signPos[0], signPos[1]
            if sy != -1:
                signRow = 2
                signColumn = -1 if sx < 50 else 1
                signColor = checkColor()
                trafficSignMatrix[currentSection%4][signRow] = signColor * signColumn
            if signRow == 2:
                switchLane(LANE_LEFT if signColor == GREEN else LANE_RIGHT)
        if signRow == -1: log.error("Empty section!!!!!")
        waitAbsLidar(0,70)
        turnCorner()
def testRun():
    initLoop()
    go(3000,0)
    waitCM(100)
    go(1000,0)
    waitCM(100)
    stop()

obstacleChallangeRun()