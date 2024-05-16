import threading
from time import sleep
from Reader import I2CReader
from log import *

COMMAND_COMMAND_STEER=6
COMMAND_SYNC=17
COMMAND_HEARTBEAT=7
COMMAND_LOG=8
COMMAND_SET_VMODE=9
COMMAND_SET_TARGETSPEED=10
CMD_SET_BREAKPERCENT=19
CMD_SET_TARGET_YAW=20
CMD_SET_SMODE=18

CMD_DATA_POSL=11
CMD_DATA_POSR=12
CMD_DATA_POSAVG=13
CMD_DATA_VMODE=14
CMD_DATA_SPEED=15
CMD_DATA_GYRO=16

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
        success=(requestData(COMMAND_SYNC))==SYNC_CODE
    log.info("sync success")
    heartbeat_thread = threading.Thread(target=beat)
    heartbeat_thread.start()
def beat():
    while True:
        sleep(0.1)
        sendCommand(COMMAND_HEARTBEAT)
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

def getAvgPos()->int:
    return requestData(CMD_DATA_POSAVG)
def setBreakForce(force:int):
    sendCommand(CMD_SET_BREAKPERCENT,force)
def setSpeed(speed:int):
    sendCommand(COMMAND_SET_TARGETSPEED,int(speed))
def setVMode(vMode:int):
    sendCommand(COMMAND_SET_VMODE,vMode)
def setSteerMode(sm:int):
    sendCommand(CMD_SET_SMODE,sm)
def getSpeed()->int:
    return requestData(CMD_DATA_SPEED)
def getVMode()->int:
    return requestData(CMD_DATA_VMODE)