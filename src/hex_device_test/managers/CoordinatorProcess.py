from typing import Optional, List
import threading
import time
from enum import Enum
import multiprocessing as mp

from .BaseCoordinator import BaseCoordinator
from ..controllers.ArmControllerProcess import ArmControllerMp as Controller
from collections import deque


class ArmCoordinator(BaseCoordinator):
    
    def __init__(self, device_ws_url_list: Optional[List[dict]] = None, enable_kcp: bool = False, arm_config: Optional[dict] = None, waypoints: Optional[List[dict]] = None, enable_view: bool = False):
        super().__init__()
        
        self._task = None
        self._enable_kcp = enable_kcp
        self._waypoints = waypoints
        self._enable_view = enable_view
        
        # 
        self._status_queue = mp.Queue(maxsize=50)
        
        self.start(device_ws_url_list, enable_kcp, arm_config)
    
    def start(self, device_ws_url_list, enable_kcp, arm_config):
        
        if device_ws_url_list is None:
            print("device ip list is None")
            return False
        
        # create controllers
        for idx, ip in enumerate(device_ws_url_list):
            controller = Controller(
                ws_url=ip,
                local_port=0,
                enable_kcp=enable_kcp,
                task_loop_hz=100,
                device_id=idx
            )
            self._controllers_list.append(controller)
        
        # event init
        print(f"controllers {len(self._controllers_list)}")

        # setting controllers
        for controller in self._controllers_list:
            controller.set_arm_config(arm_config)
            controller.set_waypoints(self._waypoints)
            controller.set_view(self._enable_view)
            # controller.set_status_callback(self._controller_status_changed)
        
        for controller in self._controllers_list:
            controller.start()
            
        self._task: threading.Thread = threading.Thread(target=self._task_loop)
        self._task.start()
        print(f"task  start")
    
    
    def shutdown(self):
        # from threading to stop controllers
        crl_shutdown_queue = deque()
        with self.controller_lock:
            for controller in self._controllers_list:
                t = threading.Thread(target=controller.shutdown)
                t.start()
                crl_shutdown_queue.append(t)      
        
        for t in crl_shutdown_queue:
            t.join(timeout=30.0)
            
        if self._task:
            self._task.join(timeout=0.1)
            self._task = None
            
        self._stop_event.set()
        print("-------------------------------- Process shutdown ----------------------------------")
    
    
    def _task_loop(self):
        try:
            task_sleep = 0.05
            while self._stop_event.is_set() == False: # condition: controllers are running
                
                pass
        except Exception as e:
            print(f"Err task loop: {e}")
            
            
    # ============ command ==============
    def publish_command(self,cmd:Optional[str] = None):
        try:
            for controller in self._controllers_list:
                controller.set_current_cmd(cmd)
            
        except Exception as e:
            print(e)
    
    # ============ callback ==============
    
    def all_brake_command(self):
        pass

    def all_home_command(self):
        pass