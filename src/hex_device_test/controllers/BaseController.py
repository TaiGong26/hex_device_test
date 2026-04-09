from abc import ABC, abstractmethod
import asyncio
from typing import Optional, List
import threading
# from collections import deque
from queue import Queue

from enum import Enum

# 继承ABC类
class BaseController(ABC):
    
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, crl_hz:int=500, device_id:int=0):
        self._async_loop = None
        self._ws_url = ws_url
        self._local_port = local_port
        self._enable_kcp = enable_kcp
        self._crl_hz = crl_hz
        self._device_id = device_id
        
        self.robot_type =None
        
        
        self._status_lock = threading.Lock()
        self._data_lock = threading.Lock()

        # data 
        self.controll_status = {}
        self.device_status = {}
        self._status_callback = None
        
        # status
        self._current_cmd = None
        self._view = False

    
    # @abstractmethod
    # def connect(self):
    #     pass
    
    @abstractmethod
    def start(self) -> bool:
        pass
    
    @abstractmethod
    def shutdown(self):
        pass
    
    # ==================== send =======================
    @abstractmethod
    def send_view_data(self):
        pass
    
    # ============= setting ===============
    def set_current_cmd(self, cmd:Optional[str]):
        with self._data_lock:
            self._current_cmd = cmd
    
    def set_status_callback(self, callback):
        self._status_callback = callback
    
    def update_status(self, new_status:dict):
        if self._status_callback is not None:
            self._status_callback(
                self._device_id, 
                new_status,
        )
    def set_view(self, view:bool):
        with self._status_lock:
            self._view = view
    
    def get_device_id(self):
        with self._status_lock:
            return self._device_id
    
    def get_device_info(self):
        with self._status_lock:
            return self.device_info
    
    # ============== judging ==============
    def judge_cmd(self, cmd:str) -> bool:
        with self._data_lock:
            if self._current_cmd == cmd:
                return True
            else:
                return False
    
    @abstractmethod
    def _task_loop(self):
        pass

    