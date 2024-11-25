from enum import Enum
import threading
from time import sleep

import serial
from Buzzer import beep_parallel
import SerialCommunicationService
from log import *

CMD_STEER=6
CMD_SYNC=17
CMD_HEARTBEAT=7
CMD_LOG=8
CMD_SET_VMODE=9
CMD_SET_TARGETSPEED=10
CMD_SET_SMODE=18
CMD_SET_BREAKPERCENT=19
CMD_SET_TARGET_YAW=20
CMD_SET_SERVO_MAX=21
CMD_SET_SERVO_MIN=22
CMD_SET_SERVO_CENT=23
CMD_SET_UNREG=25

CMD_DATA_POSL=11
CMD_DATA_POSR=12
CMD_DATA_POSAVG=13
CMD_DATA_VMODE=14
CMD_DATA_SPEED=15
CMD_DATA_GYRO=16
CMD_DATA_US=24

SYNC_CODE=18

VMODE_FORWARD=1
'''vMode (velocity), sets it so the robot may move forward'''
VMODE_BACKWARD=-1
'''vMode (velocity), sets it so the robot may move backward'''
VMODE_STOP=0
'''vMode (velocity), sets it so the robot stops (no drive on wheels)'''
VMODE_BRAKE=-2
'''vMode (velocity), sets it so the robot stops by braking (counter-drive on wheels)'''
VMODE_UNREGULATED=2
'''vMode (velocity), the motors rotate with constant power without PID speed control'''
SMODE_NONE=0
'''sMode (steer), sets it so the robot does not steer'''
SMODE_GYRO=1
'''sMode (steer), sets it so the robots steering is kept straight by the gyro'''


isSynced=False
heading=0
encoderLeft=0 #always 0 since there is no left motor
encoderRight=0 #only this encoder value is used
vMode=0
sMode=0
speed=0
logVar=0

heartbeat_thread:threading.Thread=None
ESP32LOCK=threading.Lock()
class STATE(Enum):
    HEADER=0, #Looking for packet header
    DATA=1 #Looking for data bytes
PACKET_LENGTH=26
packetBuffer=[]
serialState=STATE.HEADER
headerLetters=0
def processByte(byte):
    global serialState
    global packetBuffer
    global headerLetters
    if serialState==STATE.HEADER:
        if chr(byte)=='E':
            headerLetters=1
            # log.info("E")
        elif chr(byte)=='S' and headerLetters==1:
            headerLetters=2
            # log.info("S")
        elif chr(byte)=='P' and headerLetters==2:
            serialState=STATE.DATA
            headerLetters=0
            # log.info("P")
        packetBuffer=[]
    elif serialState==STATE.DATA:
        packetBuffer.append(byte)
        if len(packetBuffer)==PACKET_LENGTH:
            processPacket(packetBuffer)
            
            packetBuffer=[]
            serialState=STATE.HEADER
def readInt(length:int, buffer:list, delete:bool=True):
    num:int=0
    for i in range(length):
        b=int(buffer[i])
        num=(num>>(8)) | (b<<((length-1)*8))
    if num>=(2**(length*8-1)): num-=2**(length*8)
    if delete:
        del buffer[0:length]
    return num
def sendInt(data:int,data_length:int=3):
    b=bytearray(data.to_bytes(data_length,'little',signed=True))
    SerialCommunicationService.ESPserial.write(b)
def processPacket(bytes:list):
    global isSynced, heading, encoderLeft, encoderRight, vMode, speed, sMode, logVar
    # log.debug(bytes)
    isSynced=readInt(1,bytes)
    heading=readInt(4,bytes)
    encoderLeft=readInt(4,bytes)
    encoderRight=readInt(4,bytes)
    vMode=readInt(1,bytes)
    speed=readInt(4,bytes)
    sMode=readInt(1,bytes)
    logVar=readInt(4,bytes)
    if logVar>120:
        log.error("NO HB IN %s ms!"%logVar)
    # log.debug("PACKET, gyro. "%heading)
    if isSynced!=2:
        log.warn(isSynced)
        log.error("ESP not in sync!")
def sendCommand(command:int, param=None):
    ESP32LOCK.acquire()
    sendInt(command,1)
    if param!=None:
        #double check!
        sendInt(param)
    ESP32LOCK.release()
def beat():
    sendCommand(CMD_HEARTBEAT)
def getRawHeading()->int:
    return heading
def getLPos()->int:
    return encoderLeft
def getRPos()->int:
    return encoderRight
def getMotorPos()->int:
    return encoderRight
def setBrakeForce(force:int):
    sendCommand(CMD_SET_BREAKPERCENT,force)
def setSpeed(speed:int):
    sendCommand(CMD_SET_TARGETSPEED,int(speed))
def setVMode(vMode:int):
    sendCommand(CMD_SET_VMODE,vMode)
def getSMode()->int:
    return sMode
def setSteerMode(sm:int):
    sendCommand(CMD_SET_SMODE,sm)
def getSpeed()->int:
    return speed
def getVMode()->int:
    return vMode
def setServoMax(sMax):
    sendCommand(CMD_SET_SERVO_MAX,sMax)
def setServoMin(sMin):
    sendCommand(CMD_SET_SERVO_MIN,sMin)
def setServoCent(sCent):
    sendCommand(CMD_SET_SERVO_CENT,sCent)
def steer(percentage):
    sendCommand(CMD_STEER,int(percentage))
def setUnregulatedPower(power):
    sendCommand(CMD_SET_UNREG,int(power))