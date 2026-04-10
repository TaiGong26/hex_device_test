import time
from typing import Optional
import numpy as np
import traceback
import copy
from enum import Enum
from collections import deque
import multiprocessing as mp


from hex_device import HexDeviceApi
from hex_device import Arm, CommandType, Hands
from ..tools.plotjuggle import PlotjuggleDraw

from .BaseController import BaseController
from .TrajectoryController import TrajectoryPlanner


class ArmControllerMp(BaseController):
        
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, task_loop_hz:int=100, device_id:int=0):
        super().__init__(ws_url, local_port, enable_kcp, task_loop_hz, device_id)
        
        # device
        self._arm_config = None
        
        #data 
        self.data_queue = deque(maxlen=50)
        self.status_queue = None
        
        # lock
        self._status_lock = mp.Lock()
        self._data_lock = mp.Lock()

        # Task
        self._loop_running = mp.Event()
        self._task_process = None
        
        # trajectory And Home
        self._waypoints = None
        self.__HOME_POSITION = tuple([0.0, -1.5, 3.00, 0.0, 0.0, 0.0])
        self._return_home_duration = 10
        
    
    def start(self) -> bool:
        
        try: 
            
            self._loop_running.value = True
            self._task_process = mp.Process(target=self._task_loop, args=(
                self._ws_url,
                self._enable_kcp,
                self._device_id,
                self._view,
                self._loop_running,
                self._task_loop_hz,
                self._waypoints,
            ))
            self._task_process.start()

            return True
    
        except Exception as e:
            print(f"[Device {self._device_id}, Exception]: {e}")
            return False
    
    def shutdown(self):
        try:
            
            # 发送return Home
            # 等待进程的stopEvent
            # 清空所有队列和其他资源
            # 等待进程join

            self._loop_running.value = False
            self._task_process.join(timeout=20)
            
              
            print(f"[Controller {self._device_id}]: ------------------------------------ shutdown complete ------------------------------------")
        except RuntimeError as e:
            print(f"[Controller {self._device_id}, RuntimeError]: {e}")
        
        except Exception as e:
            print(f"[Controller {self._device_id}, Exception]: {e}")
        
    # ==================== setting ====================
    def set_arm_config(self, arm_config:dict):
        self._arm_config = arm_config
        self.arm_config = copy.deepcopy(arm_config)
    
    def set_waypoints(self,waypoint):
        self._waypoints = waypoint
    
    def set_view(self, view):
        self._view = view
    
    
    
    # ==================== getting ====================
    
    # ===================== judge =======================
    
    # ==================== send =======================
    # def send_view_data(
    #     self,
    #     sender:PlotjuggleDraw,
    #     device_id:int, 
    #     current_position, 
    #     target_position:Optional[list],
    #     status_mathine:Optional[Enum], 
    #     ssid:Optional[int],
    #     holder:Optional[int]
    #     ):
        
    #     device_key = f"device_id_{device_id}"
    #     data = {}

    #     # 当前值
    #     if current_position is not None and hasattr(current_position,"tolist"):
    #         current_position = current_position.tolist()

    #     # 展开 motor_position
    #     if current_position:
    #         for i, v in enumerate(current_position):
    #             data[f"{device_key}/motor_position/joint{i}"] = float(v)

    #     # 展开 target_position
    #     if target_position:
    #         for i, v in enumerate(target_position):
    #             data[f"{device_key}/target_position/joint{i}"] = float(v)

    #     # 标量
    #     if status_mathine is not None:
    #         data[f"{device_key}/status"] = status_mathine
    #     if ssid is not None:
    #         data[f"{device_key}/ssid"] = ssid
    #     if holder is not None:
    #         data[f"{device_key}/holder"] = holder

    #     sender.send_json(data)
    
    def publish_command(self,command_type:str,target_position):
        pass
    
    # ==================== task ====================

    def _task_loop(self,
        ws_url,
        enable_kcp,
        device_id,
        view,
        loop_running,
        task_hz,
        waypoints,
        ):
        
        # 声明
        task_loop_hz = 1 / task_hz
        target_position = None
        _waypoints = waypoints
        first_time = True
        _loop_running = loop_running
        is_view = view
        
        # object
        device:Arm = None
        sender = PlotjuggleDraw()
        trajectory_player:Optional[TrajectoryPlanner] = None
        return_Home = None
        
        # 初始化
        hex_api:HexDeviceApi = HexDeviceApi(
            ws_url=ws_url, 
            local_port=0, 
            enable_kcp=enable_kcp, 
        )
        start_time = time.process_time()
        
        if _waypoints:
            trajectory_player = TrajectoryPlanner(
                waypoints=_waypoints,
                segment_duration=3.0
                )
        else:
            print(f"[dev{device_id}] Not waypoints !!!!")
            return False
        
        if trajectory_player:
            res = trajectory_player.start_trajectory()
            if res:
                print(f"[dev{device_id}] trajectory player start ")
            else:
                print(f"[dev{device_id}] Not waypoints failed !!!!")

        while _loop_running.is_set() == False: # what is the condition: hex_device_api is running
            try:
                if hex_api.is_api_exit():
                    break
                
                if trajectory_player:
                    target_position = trajectory_player.get_current_target().tolist()
                
                for dev in hex_api.device_list:
                    if isinstance(dev,Arm):
                        
                        if first_time:
                            first_time = False
                            dev.start()
                        
                        send_data = {}
                        device_key = f"dev{device_id}"
                        
                        # command
                        if target_position is not None and isinstance(target_position,list):
                            dev.motor_command(CommandType.POSITION,target_position)
                            # send_data[f"{device_key}/target_position"] = target_position.copy()
                        else:
                            print(f"[dev{device_id}] target None {target_position}")
                            
                        # send_view
                        dev_position = dev.get_motor_positions(pop=False)
                        if dev_position is not None:
                            for i, v in enumerate(dev_position):
                                send_data[f"{device_key}/motor_position/joint{i}"] = float(v)
                            print(f"[dev{device_id}] Position: {dev_position}")
                        # 展开 target_position
                        if target_position is not None:
                            for i, v in enumerate(target_position):
                                send_data[f"{device_key}/target_position/joint{i}"] = float(v)
                            # print(f"[dev{device_id}] target: {target_position}")
                        
                        send_data[f"{device_key}/ssid"] = dev.get_my_session_id()
                        send_data[f"{device_key}/hold_ssid"] = dev.get_session_holder()
                        sender.send_json(send_data)
                        
                    
                # ============== status machine ==============
                
                # ============== check error ================

                # ============= update =============
                time.sleep(task_loop_hz)
                # send_view_data(target_position)
                    
                    
            except Exception as e:
                print(f"[dev {device_id}] process Exception: {e}")
                # traceback.print_exc()
            except KeyboardInterrupt:
                break
            finally:
                pass
        
        hex_api.close()
        # print(f"[dev{device_id}]: close Process")