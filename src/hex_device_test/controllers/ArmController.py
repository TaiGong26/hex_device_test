import time
import threading
from threading import BrokenBarrierError
from typing import Optional
from dataclasses import dataclass
import numpy as np
import traceback
import copy
from enum import Enum
from collections import deque
import multiprocessing as mp


from hex_device import HexDeviceApi
from hex_device import Arm, CommandType, Hands
from ..tools.plotjuggle import senders

from .BaseController import BaseController

from .TrajectoryController import TrajectoryPlanner
# from hex_device_testDemo.managers.Coordinator import Coordinator


"""
状态机
 - Disconnected
 - Init
 - Ready
 - Running
 - HoldPosition
 - Stopped
 - Error
 - LostSession
 - Exit

"""

class ArmControllerStatus(Enum):
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    Brake = 4
    Stopped = 5
    Exit = 6
    
class ArmWorkPorcess:
    def __init__(self):
        pass
    

    

class ArmController(BaseController):
        
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, task_loop_hz:int=100, device_id:int=0):
        super().__init__(ws_url, local_port, enable_kcp, task_loop_hz, device_id)
        
        # device
        self._hex_api = None
        self.device = None
        self.device_optional = None
        self.robot_type = None
        self._arm_config = None
        
        # status
        self._status_machine = ArmControllerStatus.Disconnected
        
        #data 
        self.data_queue = deque(maxlen=50)
        
        # lock
        self._status_lock = threading.Lock()
        self._data_lock = threading.Lock()

        # Task
        self._task_loop_hz = task_loop_hz
        self._loop_running = False
        self._task_thread = None
        
        # trajectory And Home
        self._waypoints = None
        self.trajectory_player: Optional[TrajectoryPlanner] = None
        self.loop_count = 0
        
        self.__HOME_POSITION = tuple([0.0, -1.5, 3.00, 0.0, 0.0, 0.0])
        self._return_home_duration = 10
        self._return_home = None
        
        self._start_time = None
        
    
    def start(self,timeout:int=3) -> bool:
        
        try: 
            self._hex_api = HexDeviceApi(
                ws_url=self._ws_url, 
                local_port=0, 
                enable_kcp=self._enable_kcp, 
                )
            
            
            for device in self._hex_api.device_list:
                print(f"[Device {self._device_id}] 发现设备: {device} type: {type(device)} robot_type: {device.robot_type}")
                if isinstance(device, Arm):
                    self.device = device
                    if device.robot_type:
                        self.robot_type = device.robot_type
                        # print(f"[Device {self._device_id}] 设备类型: {self.robot_type}")
                    break
            for device in self._hex_api.device_list:
                if isinstance(device, Hands):
                    self.device_optional = device
                    break
            
            if self.device is None:
                print("device is None")
                return False
            self.device.start()
            
            
            self._loop_running = True

            self._task_thread = threading.Thread(target=self._task_loop,daemon=True)
            self._task_thread.start()
            
            return True
    
        except Exception as e:
            print(f"[Device {self._device_id}, Exception]: {e}")
            traceback.print_exc()
            return False
    
    def shutdown(self):
        try:
            if self._task_thread:
                if self._task_thread.is_alive():
                    self._task_thread.join(timeout=0.1)
            self._task_thread = None
            
            if self._hex_api:
                self._hex_api.close()
            
            self._hex_api = None
            
            print(f"[Device {self._device_id}]: ------------------------------------ shutdown complete ------------------------------------")
        except RuntimeError as e:
            print(f"[Device {self._device_id}, RuntimeError]: {e}")
        
        except Exception as e:
            print(f"[Device {self._device_id}, Exception]: {e}")
        
    # ==================== setting ====================
    def set_arm_config(self, arm_config:dict):
        self._arm_config = arm_config
        with self._data_lock:
            self.arm_config = copy.deepcopy(arm_config)
    
    # set waypoints
    def set_waypoints(self, waypoints):
        with self._data_lock:
            self._waypoints = copy.deepcopy(waypoints)
    
    # ==================== getting ====================
    
    # ===================== judge =======================
    
    # ==================== send =======================
    
    def publish_command(self,command_type:str,target_position):
        with self._data_lock:
            if command_type == "POSITION":
                self.data_queue.append((CommandType.POSITION,target_position))
            if command_type == "BRAKE":
                self.data_queue.appendleft((CommandType.BRAKE,[0] * self.device.motor_command))
        
        # if command_type == "POSITION":
        #     self.device.motor_command(CommandType.POSITION,target_position)
        # if command_type == "BRAKE":
        #     self.device.motor_command(CommandType.BRAKE,[0] * self.device.motor_count)
            
    # def update_status(self, new_status, reason):
        # return super().update_status(new_status, reason)
    
    def update_status(self, new_status):
        return super().update_status(new_status)
    
    def send_view_data(self,target_position):
        _view = False
        with self._status_lock:
            _view = self._view
        
        if _view == True:
            device_key = f"device_id_{self._device_id}"
            data = {}

            # 当前值
            dev_position = self.device.get_motor_positions()
            if dev_position is not None:
                dev_position = dev_position.tolist()

            if target_position is not None and hasattr(target_position, "tolist"):
                target_position = target_position.tolist()

            # 展开 motor_position
            if dev_position:
                for i, v in enumerate(dev_position):
                    data[f"{device_key}/motor_position/joint{i}"] = float(v)

            # 展开 target_position
            if target_position:
                for i, v in enumerate(target_position):
                    data[f"{device_key}/target_position/joint{i}"] = float(v)

            # 标量
            data[f"{device_key}/status"] = self._status_machine
            data[f"{device_key}/ssid"] = self.device.get_my_session_id()
            data[f"{device_key}/hold_ssid"] = self.device.get_session_holder()

            senders.send_json(data)
    
    
    # ==================== task ====================

    def _task_loop(self):
        task_loop_hz = 1 / self._task_loop_hz
        target_position = None

        while self._loop_running: # what is the condition: hex_device_api is running
            try:
                
                # =========== get target position ===========
                
                # =========== send command to device ========
                
                sTime = time.perf_counter()
                
                target_data = None
                with self._data_lock:
                    if len(self.data_queue) >0:
                        target_data = self.data_queue.popleft()
                
                if target_data:
                    _type, _target = target_data
                
                if target_data:
                    self.device.motor_command(
                        _type,
                        _target
                    )
                    self.send_view_data(_target)
                    
                print(f"[dev{self._device_id}]:{self.data_queue}")
                
                eTime=time.perf_counter()
                if eTime - sTime>0.2:
                    print(f"[dev{self._device_id}] Time out")
                
                # ============== status machine ==============
                
                # ============== check error ================

                # ============= update =============
                time.sleep(task_loop_hz)
                self.send_view_data(target_position)
                    
                    
                    
            except Exception as e:
                print(f"[Device {self._device_id}] threading Exception: {e}")
                traceback.print_exc()
                # self.update_status("error",f"{e}")
                # 设置状态机
                
            # finally:
            #     print(f"[Device {self._device_id}] finally:  threading exit..")
            