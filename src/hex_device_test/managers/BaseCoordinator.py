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

# 状态机类
class StateMachine:
    
    def __init__(self):
        self._status = CoordinatorStatus.NoneStatus
        self._status_machine_lock = threading.Lock()
        
        self.cmd_queue = Queue(maxsize=50)
        
    # next status
    def set_next_status(self):
        with self._status_machine_lock:
            self._status = self._status + 1
        
    # Error status
    def set_error_status(self):
        with self._status_machine_lock:
            self._status = CoordinatorStatus.Error
        
    # Exit status
    def set_exit_status(self):
        with self._status_machine_lock:
            self._status = CoordinatorStatus.Exit
    
    # Ready status
    def set_ready_status(self):
        with self._status_machine_lock:
            self._status = CoordinatorStatus.Ready
        
    def get_current_status(self):
        with self._status_machine_lock:
            return self._status
        

class BaseCoordinator(ABC):
    def __init__(self):
        # self._async_loop = None
        # self._device_ws_url_list = _device_ws_url_list
        
        self._stop_event = threading.Event()
        self.controller_lock = threading.Lock()
        # self._send_barrier:Optional[threading.Barrier] = None
        # self._complete_action_barrier:Optional[threading.Barrier] = None
        self._send_condition = threading.Condition()
        
        self._controllers_list = deque()
        
        self._state_machine = CoordinatorStatus.NoneStatus
    
    @abstractmethod
    def start(self):
        pass
    
    @abstractmethod
    def shutdown(self):
        pass

    @abstractmethod
    def publish_command(self):
        pass
    
    @abstractmethod
    def all_brake_command(self):
        pass
    
    @abstractmethod
    def all_home_command(self):
        pass