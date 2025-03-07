from enum import Enum
from time import sleep, time
from log import *

import serial

newLidarDataFlag=False
onLidarRev=None
onLoop=None
DISTANCE_MAP:list[float]=[-1]*360
INTENSITY_MAP:list[int]=[0]*360

class STATE(Enum):
    HEADER=0,
    VERLEN=1,
    DATA=2

act_state=STATE.HEADER
errors=0
PACKET_SIZE=47
byte_packet=[0]*PACKET_SIZE
count:int=0
t0=time()



def process_packet(packet_in:list):
    global DISTANCE_MAP, newLidarDataFlag
    start_angle=packet_in[5]*256+packet_in[4]
    end_angle=packet_in[43]*256+packet_in[42]
    step:float=(end_angle-start_angle+360*1000)%36000/100/(12-1)
    for i in range(12):
        distance:float=(packet_in[7+i*3]*256+packet_in[6+i*3])/10
        intensity:int=packet_in[8+i*3]
        angle:int=round(start_angle/100+step*i)%360
        DISTANCE_MAP[angle]=distance
        INTENSITY_MAP[angle]=intensity
    if end_angle<start_angle:
        # print("ea: "+str(end_angle)+" sa: "+str(start_angle))
        newLidarDataFlag=True

def processByte(byte:int):
    global count
    global act_state
    global errors
    if act_state==STATE.HEADER:
        if byte==0x54:
            byte_packet[count]=byte
            count+=1
            act_state=STATE.VERLEN
        else:
            errors+=1
    elif act_state==STATE.VERLEN:
        if byte==0x2c:
            byte_packet[count]=byte
            count+=1
            act_state=STATE.DATA
        else:
            count=0
            act_state=STATE.HEADER
    elif act_state==STATE.DATA:
        byte_packet[count]=byte
        count+=1
        if count>=PACKET_SIZE:
            act_state=STATE.HEADER
            count=0
            process_packet(byte_packet)
LIDAR_DEADZONE_START=-130
LIDAR_DEADZONE_END=130
