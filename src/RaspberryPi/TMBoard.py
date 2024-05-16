
from rpi_TM1638 import TMBoards

# GPIO.setmode(GPIO.BOARD)
DIO = 19
CLK = 13
STB = 6, 26

TM = TMBoards(DIO, CLK, STB, 0)

def setLed(led:int,on:bool):
    TM.leds[led]=on
def set_leds(led_sequence:str="00000000"):
    for i,led_char in enumerate(led_sequence):
        TM.leds[i]=led_char=='1'
def display_data(data:int, offset:int=0, clear:bool=True):
    if clear: TM.segments[0]='        '
    TM.segments[offset]=str(data)[:8]
def displayString(string:str,offset:int=0, clear:bool=True):
    if clear: TM.segments[0]='        '
    TM.segments[offset]=string[:8]
def isPressed(key:int)->bool:
    return TM.switches[key]