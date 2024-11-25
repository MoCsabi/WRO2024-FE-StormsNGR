"""Custom logging module"""
import sys
import threading
import datetime
from pathlib import Path
import atexit
import traceback

directory = Path(__file__).parent.absolute()
"""The path of the directory the file is in"""

class main_log():
    """Logging class with different logging levels:
    * debug
    * info
    * warning
    * error
    There is also a critical that is used when an error happens on either thread
    and there is also a lidar level that saves things like lidar, gyro and camera data.
    """
    
    def log_save(self,message,level=""):
        """This is the general logging function that is used by the other level specific functions. It takes in a message a level,
        formats it and then saves it to the log file."""
        with open(Path.joinpath(directory,"logs","log.stormslog"),"a") as file:
            file.write(f"\n<{level} - section {self.section} - {datetime.datetime.now().strftime('%H:%M:%S.%f')}>\n\n{str(message)}\n")
            file.flush()
            
    def temp_save(self,message,level=""):
        """Logs the last 150 lines to a seperate temporary file for a custom vscode extension to use.
        It saves the raw text inputted line by line without extra info, only saving the logging level at the end of every line,
        which is then used to display the logs in different colors:
        * debug - blue
        * info - white
        * warning - orange
        * error - red
        * critical - dark red (although critical isn't called manually)
        """
        self.cur_data.extend([i+level for i in str(message).split("\n")])
        if len(self.cur_data) > 150:
            for i in range(len(self.cur_data)-150):
                self.cur_data.pop(i)
        with open(Path.joinpath(directory,"logs",f"log.stormslogtemp"),"w") as temp_log:
            temp_log.writelines([str(i)+"\n" for i in self.cur_data])
            temp_log.flush()
    
    
    def critical(self,message):
        """Saves and formats any message to the critcal file which is used, when an exception occurs."""
        with open(Path.joinpath(directory,"logs","critical.stormslog"),"a") as file:
            file.write(f"\n<CRITICAL - {self.section} - {datetime.datetime.now().strftime('%H:%M:%S.%f')}>\n\n{message}\n")
            file.flush()
    def debug(self,message):
        """Debug logging level, saves to both the main and temporary log files."""
        self.log_save(message,"DEBUG")
        self.temp_save(message,"__d__")
    
    def info(self,message):
        """Info logging level, saves to both the main and temporary log files."""
        self.log_save(message,"INFO")
        self.temp_save(message,"__i__")
    
    def warning(self,message):
        """Warning logging level, saves to both the main and temporary log files."""
        self.log_save(message,"WARNING")
        self.temp_save(message,"__w__")
    
    def error(self,message):
        """Error logging level, saves to both the main and temporary log files."""
        self.log_save(message,"ERROR")
        self.temp_save(message,"__e__")
    
    def lidar(self,message):
        """Logs the data of the lidar, degrees of interest, rectangles of interest, gyro position, camera data and time.
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
    """Shorter version of main_log.warning."""
    fatal = critical
    """Alternative version of main_log.critical."""
    
    
    
    def close(self,func,register = True):
        """Function that can be used to register and unregister other functions to run upon standard exit without exception."""
        if register:
            atexit.register(func)
        elif not register:
            atexit.unregister(func)
    
        
    
    def __init__(self) -> None:
        
        self.section = 0
        """Variable for the section in which the robot is in. Saved with every log."""
        
        #Variables for counting the lines of the temporary files.
        self.cur_data = []
        self.lidar_data = []
        
        #Clears all log files so every run can be separate
        open(Path.joinpath(directory,"logs","critical.stormslog"),"w").close()
        open(Path.joinpath(directory,"logs","log.stormslog"),"w").close()
        open(Path.joinpath(directory,"logs","lidar.stormslog"),"w").close()
        
        
log = main_log()

def thread_error(err):
    """Handles exceptions on threads."""
    log.critical(''.join(traceback.format_exception(err.exc_type, err.exc_value, err.exc_traceback)))
    
def main_error(a,b,c):
    """Handles exceptions on the main thread."""
    log.critical(''.join(traceback.format_exception(a,b,c)))
    
sys.excepthook = main_error
threading.excepthook = thread_error
"""Instance of main_log"""