from abc import ABC, abstractmethod
import asyncio
from typing import Optional, List
import threading
from collections import deque
from dataclasses import dataclass
from queue import Queue


# 控制器状态机
@dataclass
class CoordinatorStatus:
    NoneStatus = 0
    Init = 1
    Ready = 2
    Running = 3
    Stopped = 4
    Error = 5
    Exit = 6       


class BaseCoordinator(ABC):
    def __init__(self):
        self._stop_event = threading.Event()
        self.controller_lock = threading.Lock()
        self._send_condition = threading.Condition()
        
        self._controllers_list = deque()
        self._status_cache = {}
        
        self._state_machine = CoordinatorStatus.NoneStatus
        
        # lock 
        self._status_lock = threading.Lock()
    
    @abstractmethod
    def start(self):
        pass
    
    @abstractmethod
    def shutdown(self):
        pass

    
    def set_status(self, new_status:str):
        with self._status_lock:
            # self._status = new_status
            if new_status == "init":
                self._status = CoordinatorStatus.Init
            elif new_status == "ready":
                self._status = CoordinatorStatus.Ready
            elif new_status == "running":
                self._status = CoordinatorStatus.Running
            elif new_status == "stopped":
                self._status = CoordinatorStatus.Stopped
            elif new_status == "error":
                self._status = CoordinatorStatus.Error
            elif new_status == "exit":
                self._status = CoordinatorStatus.Exit
            else:
                print(f"invalid status: {new_status}")
                return None
            
    # ==================== status update ==================== 
    
    @abstractmethod
    def publish_command(self):
        pass
    
    # @abstractmethod
    def _controller_status_changed(
        self, 
        controller_id:int, 
        new_status:str,
        reason:str
    ):
        pass
            
            