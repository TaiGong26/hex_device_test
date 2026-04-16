from abc import ABC, abstractmethod
from typing import Optional, List
from queue import Queue
from enum import Enum

# 继承ABC类
class BaseController(ABC):
    
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, task_loop_hz:int=50, device_id:int=0):
        self._ws_url = ws_url
        self._local_port = local_port
        self._enable_kcp = enable_kcp
        self._task_loop_hz = task_loop_hz
        self._device_id = device_id
        
        self._status_lock = None
        self._data_lock = None

        # data 
        
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
    # @abstractmethod
    # def send_view_data(
    #     self,
    #     sender,
    #     device_id:int, 
    #     current_position:list, 
    #     target_position:Optional[list],
    #     status_mathine:Optional[Enum], 
    #     ssid:Optional[int],
    #     holder:Optional[int]
    #     ):
    #     pass
    
    # @abstractmethod
    # def update_status(self, new_status:dict):
    #     pass
    
    # ============= setting ===============
    # def set_current_cmd(self, cmd:Optional[str]):
    #     with self._data_lock:
    #         self._current_cmd = cmd

    def set_view(self, view:bool):
        with self._status_lock:
            self._view = view

    # ============= get =================

    # def get_device_id(self):
    #     with self._status_lock:
    #         return self._device_id
    
    # def get_device_info(self):
    #     with self._status_lock:
    #         return self.device_info
