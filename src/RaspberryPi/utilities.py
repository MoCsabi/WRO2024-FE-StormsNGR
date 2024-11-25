import threading
from typing import Iterable, Mapping
from types import GeneratorType, FunctionType
import inspect
import asyncio
import time
import atexit


class RThread(threading.Thread):
    
    def __init__(self, target: FunctionType, args: Iterable[any] = (), kwargs: Mapping[str,any] = None, name: str = None, daemon: bool = None, exit_at_end: bool = False):
        
        def result() -> any:
            return self._result
        
        def exited():
            with self._lock:
                self._get_iter = True
    
        def got_result() -> bool:
            return self._got_result
        
        def __iter__yield():
            with self._lock:
                self._get_iter = True
            while True:
                if self._got_iter:
                    return self._iter
        
        def __iter__return():
            return iter(self._result)
        
        def __nextn__():
            with self._lock:
                self._got_result = False
                self._get_result = True
            while True:
                if self._got_result:
                    return self._result
        
        def get_iterator():
            with self._lock:
                self._get_iter = True
        
        def got_iterator():
            return self._got_iter
        
        def iterator():
            return self._iter
        
        def get_next():
            with self._lock:
                self._got_result = False
                self._get_result = True
        
        def got_next():
            return self._got_result
        
        def nexter():
            return self._result
        
        if exit_at_end:
            atexit.register(exited)
        
        self._type = None
        self.target = target
        
        self._iter = None
        self._get_iter = False
        self._got_iter = False
        
        
        self.__nextn__ = None
        
        
        if inspect.isgeneratorfunction(target):
            self._type = True
        elif isinstance(target,FunctionType):
            self._type = False
        else: raise TypeError("Target is neither GeneratorType nor FunctionType")
        
        if not self._type:
            self.result = result
            self.got_result = got_result
            self.__itern__ = __iter__return
            
        if self._type:
            self.__itern__ = __iter__yield
            self.__nextn__ = __nextn__
            self.exit = exited
            self.get_iterator = get_iterator
            self.got_iterator = got_iterator
            self.iterator = iterator
            self.get_next = get_next
            self.got_next = got_next
            self.next = nexter
            
            
        
        _cur = [self]
        _cur.extend(args)
        args = _cur
        
        self._lock = threading.Lock()
        self._result = None
        self._get_result = False
        self._got_result = False
        
        
        super().__init__(target=self._handler,args=args,kwargs=kwargs,name=name,daemon=daemon)
    
    def __iter__(self):
        if self._got_iter:
            return self.result
        else:
            return self.__itern__()
    
    def __next__(self):
        if self.__nextn__ == None:
            return next(self._result)
        else:
            return self.__nextn__()
    
    def delete(self):
        del self
    

    @staticmethod
    def _handler(*args,**kwargs):
        
        async def itered():
            while True:
                if instance._get_iter:
                    with instance._lock:
                        instance._iter = iter(result)
                        instance._got_iter = True
                        break
                else:
                    await asyncio.sleep(0)
                
        async def nexted():
            while True:
                if instance._get_result:
                    with instance._lock:
                        instance._result = next(result)
                        instance._got_result = True
                        instance._get_result = False
                elif instance._got_iter:
                    break
                else:
                    await asyncio.sleep(0)
        
        async def checker():
            await asyncio.gather(itered(),nexted())
            
        
        args = list(args)
    
        instance: RThread = args.pop(0)
        func: FunctionType = instance.target

        args = tuple(args)

        result = func(*(args),**kwargs)

        
        if not instance._type:
            with instance._lock:
                instance._result = result
                instance._got_result = True
        else: asyncio.run(checker())
 

def func_thread(name: str = None, daemon: bool = None, exit_at_end:  bool = False) -> RThread:
    def inner(func: FunctionType):
        def threads(*args,**kwargs) -> RThread:
            
            thread: RThread = RThread(target=func,args=args,kwargs=kwargs, name=name, daemon=daemon, exit_at_end = exit_at_end)
            thread.start()
            return thread
        return threads
    return inner


