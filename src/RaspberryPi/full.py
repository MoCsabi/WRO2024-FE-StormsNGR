#! /usr/bin/python
'''The main python file, it contains the code for both robot runs and helper functions'''
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
from decorators import func_thread



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
#TODO: Pixy camera currently not in use
# pixy.init()
colorMatrixPrequel = [GREEN,RED]
getColorCounter = 1
#currently camera is not in use
CAM_WIDTH=315
CAM_HEIGHT=207
CAM_RIGHT_CUTOFF=CAM_WIDTH-0 #200
CAM_LEFT_CUTOFF=50
CAM_BOT_CUTOFF=CAM_HEIGHT-10
CAM_TOP_CUTOFF=20
def isCutoff(x,y):
    return x>=CAM_RIGHT_CUTOFF or x<=CAM_LEFT_CUTOFF or y>=CAM_TOP_CUTOFF or y<=CAM_BOT_CUTOFF
def checkColor(attempts:int=5)->int:
    '''Checks the color of the closest obstacle using the Pixy camera. 
    attempts: Not recommended to have over 1. Number of attempts with a delay of 0.1 seconds between if first attempt is not succesfull.'''
    global getColorCounter
    # global CAM_TOP_CUTOFF
    # global CAM_RIGHT_CUTOFF
    # global CAM_LEFT_CUTOFF
    # i=0
    # rt=-1
    # if getUS()>70:
    #     CAM_TOP_CUTOFF=20
    # else:
    #     CAM_TOP_CUTOFF=10
    # if lane==0:
    #     CAM_LEFT_CUTOFF=90
    #     CAM_RIGHT_CUTOFF=240
    # elif lane>0:
    #     CAM_LEFT_CUTOFF=0
    #     CAM_RIGHT_CUTOFF=CAM_WIDTH-150
    # elif lane<0:
    #     CAM_LEFT_CUTOFF=150
    #     CAM_RIGHT_CUTOFF=CAM_WIDTH
    getColorCounter += 1
    log.info("color: %s"%colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)])
    return colorMatrixPrequel[(getColorCounter-1)%len(colorMatrixPrequel)]
    while rt==-1 and i<attempts:
        blocks=pixy.BlockArray(4)
        n=pixy.ccc_get_blocks(4,blocks)
        log.info("detected %s blocks"%n)
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
            # rt= GREEN if b1.m_signature==1 else RED
            j=0
            while not isCutoff(blocks[j].m_x,blocks[j].m_y) and j<n: j+=1
            log.info("found block num %s"%j)
            if not isCutoff(blocks[j].m_x,blocks[j].m_y):
                rt= GREEN if blocks[j].m_signature==1 else RED
            else:
                rt=-1
                log.error("All blocks outside signature range!!")

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

lastUSDist=0

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

WARNING_DZ_TOLERANCE=3
def  readLidar(degree)->int:
    global lidarDOI
    degree=(degree+3600)%360
    if degree>180:
        degree-=360
    if degree<LIDAR_DANGERZONE_START-WARNING_DZ_TOLERANCE or degree>LIDAR_DANGERZONE_END+WARNING_DZ_TOLERANCE:
        log.warning("lidar danger zone! req. degree: %s, heading: %s" % (degree,getHeading()))
    if degree<LIDAR_DANGERZONE_START:
        degree=-LIDAR_DANGERZONE_START
    if degree>LIDAR_DANGERZONE_END:
        degree=LIDAR_DANGERZONE_END

    lidarDOI.append(degree)
    return DISTANCE_MAP[(int(degree-90)+360*100)%360]

def readAbsLidar(degree)->int:
    return readLidar((degree-getHeading())*-1)

def getAbsX()->int:
    if direction==1:
        return readAbsLidar(-90)
    else:
        return 100-readAbsLidar(90)

def getAbsY()->int:
    # return 300-readAbsLidar(0)
    return getUS()

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
LIDAR_US_DIST=24.5
'''Distance between ultrasnoic sensor and lidar'''
US_OFFSET=11
'''Ultrasonic sensor correction offset'''

def updUS():
    '''Update stored ultrasonic sensor reading'''
    global lastUSDist
    # pass
    lastUSDist=getRawUS()+LIDAR_US_DIST-US_OFFSET
    # lastUSDist=300-readAbsLidar(0)

def getUS()->float:
    '''Returns last stored ultrasonic sensor reading'''
    global lidarDOI
    lidarDOI.append(180)
    return lastUSDist
    # return 300-readAbsLidar(0)

def waitUS(cm,decreasing=True):
    '''Wait until ultrasinic sensor detects distance smaller (or larger) than cm'''
    log.info("waitUS cm %s decresing? %s"%(cm,decreasing))
    condition=True
    while condition:
        dist=getUS()
        log.debug(dist)
        if decreasing: condition=(cm<dist)
        else: condition=(cm>dist)
        sleep(0.05)

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
    log.debug("waitAbsLidar over, degree: %s, target cm: %s, actual cm: %s"%(angle,cm,lastDist))
    return lastDist

def waitCM(cm:int):
    sleep(0.01)
    p0=getAvgPos()
    log.debug("waitcm p0: %s"%p0)
    if cm>0:
        while getAvgPos()<=(p0+cm*TICKS_PER_CM): sleep(0.01)
    else:
        while getAvgPos()>=(p0+cm*TICKS_PER_CM): sleep(0.01)
    log.debug("waitcm finished, pos: %s"%getAvgPos())

def angleDiff(angle1,angle2)->int:
    diff=((angle1-angle2)+3600)%360
    if diff>180:
        diff=diff-360
    return diff
WAIT_FOR_HEADING_TOLERANCE:int=2

def waitForHeading(tolerance=None):
    if tolerance==None: tolerance=WAIT_FOR_HEADING_TOLERANCE
    while abs(angleDiff(getHeading(),pilotHeadingTarget))>tolerance:
        sleep(0.01)

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
            sleep(0.1)
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
    updUS() #update stored us reading 10 times a second
    
    # print(getHeading())
    # print("d: "+str(readAbsLidar(getHeading())))
    # updBehindCM()
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
    global DISTANCE_MAP
    DISTANCE_MAP[90]=getUS()
    lidarLogJSON["h0"]=heading0
    lidarLogJSON["heading"]=getHeading()
    lidarLogJSON["Data"]=DISTANCE_MAP
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
        displayData(readAbsLidar(0))
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

DIRECTION_RIGHT=1
DIRECTION_LEFT=-1

ARC_TO_STRAIGHT_CM=3
def setLane(wallDirection, targetDistance):
    log.info("setlane dir %s tdist %s"%(wallDirection,targetDistance))
    d0=readAbsLidar(90*wallDirection)
    arcOffset=ARC_TO_STRAIGHT_CM
    if d0<=ARC_TO_STRAIGHT_CM: arcOffset=d0/2
    if d0<targetDistance:
        go(defaultSpeed,-42*wallDirection)
        waitAbsLidar(90*wallDirection,targetDistance-arcOffset,decreasing=False)
    else:
        go(defaultSpeed,42*wallDirection)
        waitAbsLidar(90*wallDirection,targetDistance+arcOffset,decreasing=True)
    arc(0,defaultSpeed)

def switchLane(newLane:int, steep:bool=False):
    global lane
    log.info("switching lane to %s from %s ,-steep-? %s"%(newLane,lane,steep))
    if newLane==lane:
        pass
    else:
        targetDist=17+leftLaneOffset if newLane==LANE_LEFT else 83-rightLaneOffset
        if direction==DIRECTION_RIGHT:
            setLane(-1,targetDist)
        else:
            setLane(1,100-targetDist)

        lane=newLane
        
        log.info("switchlane done (at: %s from opposite wall)"%(readAbsLidar(0)))

def arc(toDegree, speed=None, percent=100):
    global pilotHeadingTarget
    log.debug("arc toDegree %s w speed %s turning percent %s"%(toDegree,speed,percent))
    if speed==None: speed=defaultSpeed
    setSteerMode(SMODE_NONE)
    if toDegree>getHeading():
        steer(copysign(percent,speed))
    else:
        steer(-copysign(percent,speed))
    setTargetSpeed(speed)
    
    pilotHeadingTarget=toDegree
    setHeadingTarget(toDegree)
    #
    
    setVMode(int(copysign(1,speed)))
    waitForHeading(tolerance=7)
    setSteerMode(SMODE_GYRO)
    waitForHeading()
    log.info("arc over at %s"%getHeading())

def turnCorner():
    global dontReverse
    global heading0
    global lane
    side=lane*direction
    if side==LANE_LEFT: # outer
        waitAbsLidar(0,40)
        arc(60,defaultSpeed)
        stop()
        arc(90,-defaultSpeed)
        
        lane=-0.5
    else: #inner
        waitAbsLidar(0,29)
        stop()
        arc(90,-defaultSpeed)
        lane=0.5
    heading0+=90*direction
    updHeading()
    log.info("turncorner over")
    dontReverse = False
    return

lastHeadingLock=threading.Lock()
def updHeading():
    global heading0
    global lastHeading
    lastHeadingLock.acquire()
    tempHeading=(requestData(CMD_DATA_GYRO))
    if tempHeading==0:
        beep_short_parallel()
        log.error("Received 0 from gyro!!")
    if tempHeading==1:
        log.error("Received 1 from gyro!!")
    if tempHeading==-1:
        log.error("Received -1 from gyro!!")
    if tempHeading==-2:
        log.error("Received -2 from gyro")
    if tempHeading!=-1:
        lastHeading=(tempHeading/10)-heading0        
    lastHeadingLock.release()

def getHeading()->int:
    lastHeadingLock.acquire()
    rt=lastHeading
    lastHeadingLock.release()
    return rt
def accelerate():
    global actSpeed
    updHeading()
    
    if actSpeed!=targetSpeed:
        if actSpeed>targetSpeed: actSpeed=(max(actSpeed-acceleration/20,targetSpeed))
        if actSpeed<targetSpeed: actSpeed=(min(actSpeed+acceleration/20,targetSpeed))
        # log.info("setting speed %s"%actSpeed)
        setSpeed(actSpeed)
    pass

RUN: bool = False

@func_thread
def checkForInput():
    global RUN
    while True:
        inputted = input()
        match inputted:
            case "run":
                RUN = True
            case "exit":
                displayString("EXIT")
                GPIO.cleanup()
                # log.logged.critical("__EXIT__")
                # os._exit(1)

def checkAngleFromWall()->int:
    leftLen=readLidar(-125)
    rightLen=readLidar(145)

def checkLidarDeadspace()->tuple[int,int]:
    leftLimit=0
    rightLimit=0
    for i in range(180):
        if DISTANCE_MAP[(i-90+3600)%360]<15 and rightLimit==0:
            rightLimit=i
        if DISTANCE_MAP[(-i-90+3600)%360]<15 and leftLimit==0:
            leftLimit=-i
    return (leftLimit,rightLimit)



#Startup display mode constants
DISPLAY_LIDAR=0
DISPLAY_LK=1
DISPLAY_ENC0=2
DISPLAY_ENC1=3
DISPLAY_IMU=4
DISPLAY_US=5
DISPLAY_LIDAR_DZ=6

#Diameter of 70
SERVO_MAX=435
SERVO_MIN=228
SERVO_SAFE_MAX=SERVO_MAX
SERVO_SAFE_MIN=228

def initLoop():
    global heading0
    global direction
    beep()
    dispMode=DISPLAY_LIDAR
    displayData(10101010)
    sync()
    checkForInput()
    setServoMin(SERVO_SAFE_MIN) #SERVO SAFE LIMIT
    setServoMax(SERVO_SAFE_MAX) #SERVO SAFE LIMIT
    # sleep(5)
    heading0=requestData(CMD_DATA_GYRO)/10
    LidarService.onLidarRev=pilotRobot
    LidarService.onLoop=accelerate
    startLidarService()

    last_qyro = getHeading()

    sleep(0.2)
    
    pressed:bool=False
    while not pressed:
        sleep(0.5)
        heading0=requestData(CMD_DATA_GYRO)/10
        if TM.switches[0] or RUN:
            setLeds("11111111")
            sleep(1)
            pressed=True
        setLed(0, last_qyro != heading0)
        # log.info("last us %s"%getRawUS())
        last_qyro = heading0
        object:tuple=(-2,-2)
        color:int=None
        if getAbsY() > 160:
            setLed(5,True)
            object=(-1,-1)
        else:
            object=findNearestPointAbs((30,175),(70,225))
            if object[1] > -1:
                setLed(5, False)
                color=colorMatrixPrequel[0]
                # color = checkColor()
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
        if dispMode==DISPLAY_LIDAR:
            displayData("L.",readLidar(0))
        elif dispMode==DISPLAY_ENC0:
            displayData("E0.",getLPos())
        elif dispMode==DISPLAY_ENC1:
            displayData("E1.",getRPos())
        elif dispMode==DISPLAY_IMU:
            displayData("I.",requestData(CMD_DATA_GYRO))
        elif dispMode==DISPLAY_LK:
            displayString("88888888")
            setLeds("11111111")
        elif dispMode==DISPLAY_US:
            displayData("U.",getUS())
        elif dispMode==DISPLAY_LIDAR_DZ:
            limits=checkLidarDeadspace()
            displayString("D.%s  %s"%(limits[0],limits[1]))
        if TM.switches[1]:
            displayString("--------")
            sleep(0.2)
            dispMode=(dispMode+1)%6
        # if TM.switches[7]:
        #     displayString("EXIT")
        #     log.error("EXIT")
        #     GPIO.cleanup()
            
        #     os._exit(0)
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

currentSection = 0
def findFirst(section):
    colorPos=0
    for i in range(3):
        if trafficSignMatrix[section][i]!=0:
            log.info("findfirst %s"%trafficSignMatrix[section][i])
            return trafficSignMatrix[section][i],i

def findLast(section):
    colorPos=0
    for i in range(3):
        if trafficSignMatrix[section][2-i]!=0: return trafficSignMatrix[section][2-i],i


dontReverse = False
def turnAround():
    global dontReverse
    global heading0
    global doDetection
    doDetection=False
    if lane==LANE_RIGHT:

        waitAbsLidar(0,70)
        arc(-70,defaultSpeed)
        waitAbsLidar(-90,20)
        go(-defaultSpeed,-90)
        waitAbsLidar(-90,40,decreasing=False)
        stop()
        go(1000,-180)
        waitForHeading()
        dontReverse = True
        heading0 -= 180
        updHeading()

parkPos=-1

doDetection=True
defaultSpeed=1000
def reverseMatrix():
    global trafficSignMatrix
    global parkPos
    parkPos=3-parkPos
    #TODO:PARKPOS ÁTGONDOL
    trafficSignMatrix.reverse()
    for i in range(len(trafficSignMatrix)):
        trafficSignMatrix[i].reverse()
        for j in range(len(trafficSignMatrix[i])):
            trafficSignMatrix[i][j]*=-1
rightLaneOffset, leftLaneOffset=0,0
startTime=-1
def obstacleChallangeRun():
    global direction
    global startTime
    log.info("START")
    color, object = initLoop()
    startTime=time()
    log.info("Direction is %s"%direction)
    global trafficSignMatrix
    global currentSection
    global doDetection
    global dontReverse
    global leftLaneOffset
    global rightLaneOffset
    global parkPos
    if direction==DIRECTION_LEFT:
        parkObj=findNearestPointAbs((80,190),(90,210))
        px, py=parkObj[0], parkObj[1]
        if py!=-1: rightLaneOffset=20
    if object[1]==-1:
        switchLane(LANE_LEFT*direction)
    else:
        if color == GREEN:
            switchLane(LANE_LEFT,False)
            
        elif color == RED:
            switchLane(LANE_RIGHT,False)
        trafficSignMatrix[currentSection%4][2]=color*(-1 if object[0] < 50 else 1)
    turnCorner()

    i=1
    while i<13:
    # for i in range(1,13):

        section=i
        if parkPos==-1:
            if direction==DIRECTION_LEFT:
                parkObj=findNearestPointAbs((80,150),(90,210))
            else:
                parkObj=findNearestPointAbs((10,90),(20,150))
            px,py=parkObj[0],parkObj[1]
            if py!=-1: parkPos=section%4
        if parkPos%4==section%4 and parkPos!=-1:
            if direction==DIRECTION_RIGHT:
                leftLaneOffset=20
            else:
                rightLaneOffset=20
        else:
            leftLaneOffset=0
            rightLaneOffset=0
        currentSection = i
        log.section = currentSection
        if not dontReverse:
            go(-defaultSpeed,0)
            waitUS(50)
            # waitAbsLidar(0,250,decreasing=False)
        if doDetection:
            #DETECTION

            
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
            if currentSection== 8:
                doDetection = False
        if not dontReverse:
            waitUS(40)
            # waitAbsLidar(0,260,decreasing=False)
            stop()

        go(defaultSpeed,0)
        if doDetection:
            #STILL DETECT

            if signRow != -1:
                switchLane(LANE_LEFT if signColor == GREEN else LANE_RIGHT)
            
            if signRow != 1:
                waitUS(120,decreasing=False)
                # waitAbsLidar(0,180)
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

        else:
            #OPTIMIZED - NO DETECTION
            switchLane(LANE_LEFT if abs(findFirst(currentSection%4)[0]) == GREEN else LANE_RIGHT)
            if currentSection==12:
                waitAbsLidar(0,135)
                stop()
                log.error("FINISHED WITH TIME %s"%(time()-startTime))
                os._exit(0)
            if trafficSignMatrix[currentSection%4][2]!=0:
                waitAbsLidar(0,180)
                switchLane(LANE_LEFT if abs(trafficSignMatrix[currentSection%4][2]) == GREEN else LANE_RIGHT)
        #JOINED
        if readAbsLidar(0)>150:
            # setTargetSpeed(3000)
            pass
        waitAbsLidar(0,90)
        setTargetSpeed(defaultSpeed)
        
        if currentSection==7 and abs(findLast(3)[0])==RED:
            reverseMatrix()
            direction*=-1
            i+=1
            turnAround()
        else:

            turnCorner()
        i+=1
    

def testRun():
    global pilotHeadingTarget
    global trafficSignMatrix
    
    initLoop()
    # beepPWM(440)
    log.info(checkLidarDeadspace())

    # pilotHeadingTarget=180
    # l1=readAbsLidar(-90)
    # waitForHeading()
    # pilotHeadingTarget=360
    # l2=readAbsLidar(90)
    # display_data((l2-l1))

obstacleChallangeRun()
stop()