import os
import sys
import threading
import datetime
from pathlib import Path
import atexit
import traceback

directory = Path(__file__).parent.absolute()


class main_log():
    
    def log_save(self,message,level=""):
        with open(Path.joinpath(directory,"logs","log.stormslog"),"a") as file:
            file.write(f"\n<{level} - section {self.section} - {datetime.datetime.now().strftime('%H:%M:%S.%f')}>\n\n{str(message)}\n")
            file.flush()
            
    def temp_save(self,message,level=""):
        """Logs the last 150 lines to a seperate temporary file for a custom vscode extension to use.
        It saves the raw text inputed line by line without extra info only saving the logging level at the end of every line."""
        self.cur_data.extend([i+level for i in str(message).split("\n")])
        if len(self.cur_data) > 150:
            for i in range(len(self.cur_data)-150):
                self.cur_data.pop(i)
        with open(Path.joinpath(directory,"logs",f"log.stormslogtemp"),"w") as temp_log:
            temp_log.writelines([str(i)+"\n" for i in self.cur_data])
            temp_log.flush()
    
    
    def critical(self,message):
        with open(Path.joinpath(directory,"logs","critcal.stormslog"),"a") as file:
            file.write(f"\n<CRITICAL - {self.section} - {datetime.datetime.now().strftime('%H:%M:%S.%f')}>\n\n{message}\n")
            file.flush()
    
    def thread_error(self,err):
        self.critical(''.join(traceback.format_exception(err.exc_type, err.exc_value, err.exc_traceback)))
    
    def main_error(self,a,b,c):
        self.critical(''.join(traceback.format_exception(a,b,c)))
    
    def debug(self,message):
        self.log_save(message,"DEBUG")
        self.temp_save(message,"__d__")
    
    def info(self,message):
        self.log_save(message,"INFO")
        self.temp_save(message,"__i__")
    
    def warning(self,message):
        self.log_save(message,"WARNING")
        self.temp_save(message,"__w__")
    
    def error(self,message):
        self.log_save(message,"ERROR")
        self.temp_save(message,"__e__")
    
    def lidar(self,message):
        """Logs the data of the lidar, degrees of interest, rectangles of interest, gyro position and time.
        Saves to a seperate file from the rest. Also saves to a temporary file where only the last 2 data are saved."""
        
        with open(Path.joinpath(directory,"logs","lidar.stormslog"),"a") as file:
            file.write(message + "\n")
            file.flush()
        
        self.lidar_data.extend(message.split("\n"))
        if len(self.lidar_data) > 2:
            for i in range(len(self.lidar_data)-2):
                self.lidar_data.pop(i)
        with open(Path.joinpath(directory,"logs",f"lidar.stormslogtemp"),"w") as temp_log:
            temp_log.writelines([str(i)+"\n" for i in self.lidar_data])
            temp_log.flush()
    
    warn = warning
    fatal = critical
    
    sys.excepthook = main_error
    threading.excepthook = thread_error
    
    def close(self,func,register = True):
        if register:
            atexit.register(func)
        elif not register:
            atexit.unregister(func)
    
        
    
    def __init__(self) -> None:
        
        self.section = 0
        self.cur_data = []
        self.lidar_data = []
        
        open(Path.joinpath(directory,"logs","critical.stormslog"),"w").close()
        open(Path.joinpath(directory,"logs","log.stormslog"),"w").close()
        open(Path.joinpath(directory,"logs","lidar.stormslog"),"w").close()
        
        
log = main_log()




