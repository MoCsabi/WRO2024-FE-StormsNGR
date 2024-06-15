'''Threading with decorators'''
from time import sleep
import threading

def func_thread(func):
    def threads(*args,**kwargs) -> threading.Thread:
        thread: threading.Thread = threading.Thread(target=func,args=args,kwargs=kwargs)
        thread.start()
        return thread
    return threads