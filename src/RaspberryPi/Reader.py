import serial
from smbus2 import SMBus


class SerialReader:
    serialC:serial.Serial=None
    def __init__(self,port="USB0",baud=115200):
        self.serialC=serial.Serial(port="/dev/tty"+port,baudrate=baud)
        self.serialC.reset_input_buffer()
        self.serialC.reset_output_buffer()
    def readInt(self,bytes:int) -> int:
        num:int=0
        for _ in range(bytes):
            b=int(self.serialC.read()[0])
            num=(num>>(8)) | (b<<((bytes-1)*8))
        if num>=(2**(bytes*8-1)): num-=2**(bytes*8)
        return num
    def readUnsignedInt(self) -> int:
        return int(self.serialC.read()[0])
    def sendUnsignedInt(self,command):
        self.serialC.write(command)
    def sendInt(self,data:int,data_length:int):

        b=bytearray(data.to_bytes(data_length,'little',signed=True))
        # print(b)
        self.serialC.write(b)
        # for i in range(data_length):
        #     # self.serialC.write(data>>(8*i) & 255)
        #     self.serialC.write(int((data>>(8*i) & 255)))

class I2CReader:
    address:int=0x0
    bus:SMBus=None
    def __init__(self,address=0x8, channel=1):
        self.bus=SMBus(channel)
        self.address=address
    def readInt(self, register:int=100) -> int:
        num:int=0
        data=self.bus.read_block_data(self.address,register)
        data=self.bus.read_block_data(self.address,register)
        length=len(data)
        for val in data:
            num=(num>>(8)) | (val<<((length-1)*8))
        if num>=(2**(length*8-1)): num-=2**(length*8)
        return num
    def writeByte(self,byte:int):
        self.bus.write_byte(i2c_addr=self.address,value=byte)
    def sendInt(self,data:int,register:int):
        b=bytearray(data.to_bytes(3,'little',signed=True))
        # print(b)
        self.bus.write_block_data(self.address,register,b)
