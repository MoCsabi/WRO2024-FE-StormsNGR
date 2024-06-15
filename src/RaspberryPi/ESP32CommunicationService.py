import threading
from time import sleep
from Reader import I2CReader
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

CMD_DATA_POSL=11
CMD_DATA_POSR=12
CMD_DATA_POSAVG=13
CMD_DATA_VMODE=14
CMD_DATA_SPEED=15
CMD_DATA_GYRO=16
CMD_DATA_US=24

SYNC_CODE=18


heartbeat_thread:threading.Thread=None
bus=I2CReader()
ESP32LOCK=threading.Lock()

def sendCommand(command:int, param=None, await_response=False):
    ESP32LOCK.acquire()
    if param!=None:
        bus.sendInt(param,command)
    else:
        bus.writeByte(byte=command)
    if await_response:
        rt_val=bus.readInt()
        ESP32LOCK.release()
        return rt_val
    ESP32LOCK.release()
def getResponse() -> int:
    return bus.readInt()
def sync():
    global heartbeat_thread
    success=False
    while not success:
        log.warning("sync attempt failed")
        success=(requestData(CMD_SYNC))==SYNC_CODE
    log.info("sync success")
    heartbeat_thread = threading.Thread(target=beat)
    heartbeat_thread.start()
def beat():
    while True:
        sleep(0.1)
        sendCommand(CMD_HEARTBEAT)
def requestData(command:int) -> int:
    ESP32LOCK.acquire()
    rt=bus.readInt(command)
    ESP32LOCK.release()
    return rt
i2cDict:dict=dict()
def getData(command:int) ->int:
    return i2cDict[command]
def updData(command:int):
    # if i2cDict[command]==None:
    #     pass
        # i2cDict
    i2cDict[command]=requestData(command)
def getLPos()->int:
    return requestData(CMD_DATA_POSL)
def getRPos()->int:
    return requestData(CMD_DATA_POSR)
def getAvgPos()->int:
    return requestData(CMD_DATA_POSAVG)
def setBreakForce(force:int):
    sendCommand(CMD_SET_BREAKPERCENT,force)
def setSpeed(speed:int):
    sendCommand(CMD_SET_TARGETSPEED,int(speed))
def setVMode(vMode:int):
    sendCommand(CMD_SET_VMODE,vMode)
def setSteerMode(sm:int):
    sendCommand(CMD_SET_SMODE,sm)
def getSpeed()->int:
    return requestData(CMD_DATA_SPEED)
def getVMode()->int:
    return requestData(CMD_DATA_VMODE)
def setServoMax(sMax):
    sendCommand(CMD_SET_SERVO_MAX,sMax)
def setServoMin(sMin):
    sendCommand(CMD_SET_SERVO_MIN,sMin)
def setServoCent(sCent):
    sendCommand(CMD_SET_SERVO_CENT,sCent)
def steer(percentage):
    sendCommand(CMD_STEER,int(percentage))
def getRawUS()->float:
    return requestData(CMD_DATA_US)/100