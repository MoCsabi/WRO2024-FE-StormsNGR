import threading
from time import sleep, time

import serial

from Buzzer import beep_twice_parallel
import ESP32_Service
import LidarService
from log import log

accelFunc=None
lidarSerial = serial.Serial(
    port="/dev/ttyAMA0", baudrate=230400, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity="N",
)
ESPserial = serial.Serial(
    port="/dev/ttyUSB0", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, parity="N",
)
def startReading():
    # lidarSerial.reset_input_buffer()
    readThread = threading.Thread(target=readData)
    readThread.start()
    sleep(0.2)

def readData(): #Runs constantly, checks for new data and calls the correct process function. Also handles acceleration.
    t0=time()
    t00=time()
    accT0=time()
    while True:
        if time()-t00>0.1:
            beep_twice_parallel()
            log.error("Comm. thread too much time elapsed! dTime %s"%(time()-t00))
        t00=time()
        if lidarSerial.in_waiting>0:
            LidarService.processByte((lidarSerial.read()[0]))
        
        if ESPserial.in_waiting>0:
            ESP32_Service.processByte(ESPserial.read()[0]) 
        if time()-t0>0.1:
            ESP32_Service.beat()
            t0=time()
        if time()-accT0>0.05:
            accelFunc()
            accT0=time()
        
        # log.debug("c")