import logging
import sys
import os
from pathlib import Path
import threading

logging.LIDAR = logging.DEBUG + 5


logging.addLevelName(logging.LIDAR,"LIDAR")

def rename_lidar(self, message, *args, **kws):
    if self.isEnabledFor(logging.LIDAR):
        self._log(logging.LIDAR , message, args, **kws)

logging.Logger.lidar = rename_lidar

#--------------------------------------------------


class specific_level_filter(logging.Filter):
    def __init__(self,level):
        super().__init__()
        self.level = level
    
    def filter(self, record):
        return record.levelno == self.level



class main_log():
    
    directory = Path(__file__).parent.absolute()
    logged = logging.getLogger("MainLog")
    logged.propagate = False
    logged.setLevel(logging.DEBUG)

    section = 0
    date = "%H:%M:%S"
    
    cur_data = []
    lidar_data = []
    
    def trace_error(a,b,c):
        main_log.logged.critical("",exc_info=b)
    
    def thread_error(err):
        main_log.logged.critical("",exc_info=err)
    
    threading.excepthook = thread_error
    sys.excepthook = trace_error

    def make_handler(self,level,file="log",format=f'\n<%(levelname)s - section {section} - %(asctime)s>\n%(message)s\n'):
        self.handler = logging.FileHandler(Path.joinpath(self.directory,"logs",f"{file}.stormslog"),mode="w")
        self.handler.setLevel(level)
        self.handler_format = logging.Formatter(format,self.date)
        self.handler.setFormatter(self.handler_format)
        self.handler.addFilter(specific_level_filter(level))
        self.logged.addHandler(self.handler)
        
    def log_message(self,message):
        self.cur_data.extend(message.split("\n"))
        if len(self.cur_data) > 150:
            for i in range(len(self.cur_data)-150):
                self.cur_data.pop(i)
        with open(Path.joinpath(self.directory,"logs",f"log.stormslogtemp"),"w") as temp_log:
            temp_log.writelines([f"{i}\n" for i in self.cur_data])
        
    def debug(self,message):
        self.logged.debug(message)
        self.log_message(message)
    
    def info(self,message):
        self.logged.info(message)
        self.log_message(message)
    
    def warning(self,message):
        self.logged.warning(message)
        self.log_message(message)
    
    def error(self,message):
        self.logged.error(message)
        self.log_message(message)
    
    def lidar(self,message):
        self.logged.lidar(message)
        
        self.lidar_data.extend(message.split("\n"))
        if len(self.lidar_data) > 2:
            for i in range(len(self.lidar_data)-2):
                self.lidar_data.pop(i)
        with open(Path.joinpath(self.directory,"logs",f"lidar.stormslogtemp"),"w") as temp_log:
            temp_log.writelines([f"{i}\n" for i in self.lidar_data])
        
    

    def __init__(self):
        os.makedirs(self.directory / "logs", exist_ok=True)
        self.critical_handler = logging.FileHandler(Path.joinpath(self.directory,"logs","critical.stormslog"),mode="w")
        self.critical_handler.setLevel(logging.CRITICAL)
        self.critical_format = logging.Formatter('\n<%(levelname)s - %(asctime)s>\n%(message)s\n',self.date)
        self.critical_handler.setFormatter(self.critical_format)
        self.logged.addHandler(self.critical_handler)
        
        
        self.make_handler(logging.DEBUG)
        self.make_handler(logging.INFO)
        self.make_handler(logging.WARNING)
        self.make_handler(logging.ERROR)
        
        '''Custom lidar logging level it's level is logging.DEBUG + 5'''
        
        self.make_handler(logging.LIDAR,"lidar","%(message)s")

log = main_log()