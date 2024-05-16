#! /usr/bin/python
import threading
from time import sleep
from RPi import GPIO
BUZZ_PIN=23
GPIO.setup(BUZZ_PIN,GPIO.OUT)
GPIO.output(BUZZ_PIN,GPIO.LOW)
def beep():
    GPIO.output(BUZZ_PIN,GPIO.HIGH)
    sleep(0.5)
    GPIO.output(BUZZ_PIN,GPIO.LOW)
def beep_parallel():
    b_t=threading.Thread(target=beep)
    b_t.start()
