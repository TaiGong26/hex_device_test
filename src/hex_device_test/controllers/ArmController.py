import time
import threading
from threading import BrokenBarrierError
from typing import Optional
from dataclasses import dataclass
import numpy as np
import traceback
import copy


from hex_device import HexDeviceApi
from hex_device import Arm, CommandType, Hands

from .BaseController import BaseController
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
 - Exit

"""

class TrajectoryPlanner:
    """Trajectory planner that supports smooth acceleration and deceleration planning"""
    
    def __init__(self, waypoints, segment_duration=3.0):
        """
        Initialize trajectory planner
        waypoints: List of waypoints
        segment_duration: Duration of each trajectory segment (seconds)
        """
        self.waypoints = waypoints
        self.segment_duration = segment_duration
        
        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.start_time = None
        self.last_target_position = None  # Store last commanded position
        
    def start_trajectory(self):
        """Start trajectory execution"""
        if not self.waypoints:
            return False
            
        self.trajectory_started = True
        self.start_time = time.time()
        self.current_waypoint_index = 0
        return True
        
    def get_current_target(self):
        """Get the target position at the current moment"""
        if not self.trajectory_started or not self.waypoints:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        total_segments = len(self.waypoints)
        segment_index = int(elapsed_time / self.segment_duration) % total_segments
        
        segment_elapsed = elapsed_time % self.segment_duration
        normalized_time = segment_elapsed / self.segment_duration
        
        start_waypoint = self.waypoints[segment_index]
        end_waypoint = self.waypoints[(segment_index + 1) % total_segments]
        
        # Use S-curve interpolation to calculate current position
        s = self._smooth_step(normalized_time)
        
        start_pos = np.array(start_waypoint)
        end_pos = np.array(end_waypoint)
        target_position = start_pos + s * (end_pos - start_pos)
        
        self.current_waypoint_index = segment_index
        self.last_target_position = target_position  # Store for potential return home
        
        return target_position
    
    def get_last_position(self):
        """Get the last commanded position"""
        return self.last_target_position
        
    def _smooth_step(self, t):
        """S-curve interpolation function that provides smooth acceleration and deceleration"""
        # Limit t to [0,1] range
        t = max(0.0, min(1.0, t))
        
        # Use 5th degree polynomial for smoother interpolation: 6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3
        
    def get_current_segment_info(self):
        """Get information about the current segment"""
        if not self.trajectory_started:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        segment_index = int(elapsed_time / self.segment_duration) % len(self.waypoints)
        segment_elapsed = elapsed_time % self.segment_duration
        segment_progress = segment_elapsed / self.segment_duration
        
        return {
            'segment_index': segment_index,
            'segment_progress': segment_progress,
            'total_elapsed': elapsed_time
        }


class ReturnHomeController:
    """Controller for smooth return to home position"""
    
    def __init__(self, start_position, home_position, duration):
        """
        Initialize return home controller
        start_position: Starting position (current position when Ctrl+C is pressed)
        home_position: Target home position
        duration: Duration to reach home position (seconds)
        """
        self.start_position = np.array(start_position)
        self.home_position = np.array(home_position)
        self.duration = duration
        self.start_time = time.time()
        
    def get_target_position(self):
        """Get the current target position during return home"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time >= self.duration:
            return self.home_position, True  # Reached home
        
        # Calculate normalized time [0, 1]
        t = elapsed_time / self.duration
        
        # Use S-curve interpolation for smooth motion
        s = self._smooth_step(t)
        
        # Interpolate between start and home position
        target_position = self.start_position + s * (self.home_position - self.start_position)
        
        return target_position, False  # Not yet reached home
    
    def _smooth_step(self, t):
        """S-curve interpolation function"""
        t = max(0.0, min(1.0, t))
        return 6 * t**5 - 15 * t**4 + 10 * t**3


class ArmController(BaseController):
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, crl_hz:int=500, device_id:int=0,task_loop_hz:int=100):
        super().__init__(ws_url, local_port, enable_kcp, crl_hz, device_id)
        
        self._hex_api = None
        self.device = None
        self.device_optional = None
        self._task_thread = None
        self.start_thread=threading.Thread(target=self.start,daemon=True)
        # self.status = ArmControllerStatus.Disconnected
        
        self._task_loop_hz = task_loop_hz
        self.robot_type = None
        self._arm_config = None
        self._waypoints = None
        self._current_cmd = None
        
        self.trajectory_player: Optional[TrajectoryPlanner] = None
        self.loop_count = 0
        
        self._task_thread = None
        
        self._home_position = tuple([0.0, -1.5, 3.00, 0.0, 0.0, 0.0])
        self._return_home_duration = 10
        self._return_home = None
        
        # self._controll_info = {
        #     "trajectory_started": False,
        #     "current_waypoint_index": 0,
        #     "last_target_position": None,
        #     "status" : None,
        #     "current_cmd": None,
        #     "loop_count": 0,
        # }
        
    # def connect(self):
    #     pass
    
    def start(self) -> bool:
        
        self._hex_api = HexDeviceApi(
            ws_url=self._ws_url, 
            local_port=0, 
            enable_kcp=self._enable_kcp, 
            )
        
        # add wait for device list to be updated
        while not self._hex_api.device_list:
            time.sleep(0.1)
            
        for device in self._hex_api.device_list:
            print(f"[Device {self._device_id}] 发现设备: {device} type: {type(device)}")
            if isinstance(device, Arm):
                self.device = device
                # with self._status_lock:
                    # self.status = ArmControllerStatus.Init
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
        
        # with self._status_lock:
            # self.status = ArmControllerStatus.Ready
        
        if self._waypoints:
            self.trajectory_player = TrajectoryPlanner(
                waypoints=self._waypoints,
                segment_duration=3.0
                )
        else:
            print(f"[Device {self._device_id}] Not waypoints !!!!")
            return False
        
        if self.trajectory_player:
            
            res = self.trajectory_player.start_trajectory()
            if res:
                print(f"[Device {self._device_id}] trajectory player start ")
            else:
                print(f"[Device {self._device_id}] Not waypoints failed !!!!")
                
        # move to home position
        
        self._task_thread = threading.Thread(target=self._task_loop,daemon=True)
        self._task_thread.start()
        
        return True
   
    def shutdown(self):
        try:
            # wait all home !!!!!
            
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

    def set_current_cmd(self, cmd:Optional[str]):
        with self._data_lock:
            self._current_cmd = cmd
    
    # ==================== getting ====================
    def get_status(self):
        pass
    
    def get_thread_is_alive(self):
        pass
    
    # ==================== task ====================
    
    def _move_to_home():
        pass
    
    def _task_loop(self):
        task_loop_hz = 1 / self._task_loop_hz
        while True: # what is the condition: hex_device_api is running
            try:
                    
                        
                    target_position = None
                    if self.trajectory_player:
                        target_position = self.trajectory_player.get_current_target().tolist()
                    
                    
                    if target_position:
                        self.device.motor_command(
                            CommandType.POSITION,
                            target_position)
                    
                    # ============= test ===========
                    # print(f"[Device {self._device_id}] target_position: {target_position} motor position: {self.device.get_motor_positions()}")
                    
                    time.sleep(task_loop_hz)
                    
                    
                    
            except Exception as e:
                print(f"[Device {self._device_id}] threading Exception: {e}")
                traceback.print_exc()
                # event delegation
            except BrokenBarrierError as e:
                print(f"[Device {self._device_id}] Waring Barrier broken")
                # event delegation
            # finally:
            #     print(f"[Device {self._device_id}] finally:  threading exit..")
            