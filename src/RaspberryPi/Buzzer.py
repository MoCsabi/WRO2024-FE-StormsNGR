#! /usr/bin/python
import threading
from time import sleep
from RPi import GPIO
BUZZ_PIN=5
BUZZ_HZ=1000
GPIO.setup(BUZZ_PIN,GPIO.OUT)
GPIO.output(BUZZ_PIN,GPIO.LOW)
pwm = GPIO.PWM(BUZZ_PIN, BUZZ_HZ)
def beep_short():
    GPIO.output(BUZZ_PIN,GPIO.HIGH)
    sleep(0.2)
    GPIO.output(BUZZ_PIN,GPIO.LOW)
def beep():
    GPIO.output(BUZZ_PIN,GPIO.HIGH)
    sleep(0.5)
    GPIO.output(BUZZ_PIN,GPIO.LOW)
def beep_twice():
    GPIO.output(BUZZ_PIN,GPIO.HIGH)
    sleep(0.4)
    GPIO.output(BUZZ_PIN,GPIO.LOW)
    sleep(0.3)
    GPIO.output(BUZZ_PIN,GPIO.HIGH)
    sleep(0.4)
    GPIO.output(BUZZ_PIN,GPIO.LOW)
def beep_parallel():
    b_t=threading.Thread(target=beep)
    b_t.start()
def beep_twice_parallel():
    b_t=threading.Thread(target=beep_twice)
    b_t.start()
def beep_short_parallel():
    b_t=threading.Thread(target=beep_short)
    b_t.start()
def beepPWM(hz):
    pwm.ChangeFrequence(hz)
    pwm.start(50)
    sleep(100)
    pwm.stop()