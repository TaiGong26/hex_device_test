from abc import ABC, abstractmethod
import asyncio
from typing import Optional, List
import threading
from collections import deque
from dataclasses import dataclass
from queue import Queue
from enum import Enum



# 控制器状态机
class CoordinatorStatus(Enum):
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    Stopped = 4
    Error = 5


class BaseCoordinator(ABC):
    def __init__(self):
        self._stop_event = threading.Event()
        self.controller_lock = threading.Lock()
        self._send_condition = threading.Condition()
        
        self._controllers_list = deque()
        self._status_queue = None
        
        self._state_machine = CoordinatorStatus.Disconnected
        
        # lock 
        self._status_lock = threading.Lock()
    
    @abstractmethod
    def start(self):
        pass
    
    @abstractmethod
    def shutdown(self):
        pass

    # ==================== status update ==================== 
    
    @abstractmethod
    def publish_command(self):
        pass
            