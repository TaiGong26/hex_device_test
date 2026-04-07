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
from ..tools.plotjuggle import senders

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

class ArmControllerStatus:
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    HoldPosition = 4
    Stopped = 5
    Exit = 6
    Error = 7


class TrajectoryController:
    def __init__(self):
        pass
    
    def get_target_position(self,current_position):
        # 传递当前位姿，计算下一时刻的目标位姿；可以选择不传递，由控制器自己维护状态
        pass
    
    def startController(self):
        pass
    
    # def set_current_position(self, position):
    #     pass
    
    # 平滑
    def _smooth_step(self, t):
        """S-curve interpolation function that provides smooth acceleration and deceleration"""
        # Limit t to [0,1] range
        t = max(0.0, min(1.0, t))
        
        # Use 5th degree polynomial for smoother interpolation: 6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3

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
        # self.status = ArmControllerStatus.Disconnected
        
        self._task_loop_hz = task_loop_hz
        self.robot_type = None
        self._arm_config = None
        self._waypoints = None
        self._current_cmd = None
        self._view = False
        
        self.trajectory_player: Optional[TrajectoryPlanner] = None
        self.loop_count = 0
        
        self._task_thread = None
        
        self._home_position = tuple([0.0, -1.5, 3.00, 0.0, 0.0, 0.0])
        self._return_home_duration = 10
        self._return_home = None
        
        self._start_time = None
        
        self._status_machine = ArmControllerStatus.Disconnected
        
        self.__HOME_POSITION = tuple([0.0, -1.5, 3.00, 0.0, 0.0, 0.0])
        
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
    
    def start(self,timeout:int=3) -> bool:
        
        try: 
        
            self._hex_api = HexDeviceApi(
                ws_url=self._ws_url, 
                local_port=0, 
                enable_kcp=self._enable_kcp, 
                )
            
            # add wait for device list to be updated
            start_time= time.time()
            while not self._hex_api.device_list:
                if time.time() - start_time > timeout:
                    print(f"[Device {self._device_id}] Timeout waiting for device list")
                    return False
                time.sleep(0.05)

            print(f"[Device {self._device_id}] HexDeviceApi created at {id(self._hex_api)}")

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
            
            ssid = self.device.get_my_session_id()
            hold_ssid = self.device.get_session_holder()
            while not ssid or not hold_ssid or ssid != hold_ssid:
                # if time.time() - start_time > timeout:
                print(f"[Device {self._device_id}] waiting for session hold:  myssid:{ssid} holdssid:{hold_ssid}")   
                ssid = self.device.get_my_session_id()
                hold_ssid = self.device.get_session_holder()
                time.sleep(0.05)
                
            self.set_status("init")
            print(f"[Device {self._device_id}] device session hold")
            
            
            
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
                    
            # self._start_time = time.time()

            while device.get_session_holder() != device.get_my_session_id():
                time.sleep(0.1)
                print(f"[Device {self._device_id}] waiting for session hold:  myssid:{ssid} holdssid:{hold_ssid}")
            

            self._task_thread = threading.Thread(target=self._task_loop,daemon=True)
            self._task_thread.start()
            
            return True
    
        except Exception as e:
            print(f"[Device {self._device_id}, Exception]: {e}")
            traceback.print_exc()
            return False
    
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
            
    def set_status(self, status:str):
        with self._status_lock:
            if status == "init":
                self._status_machine = ArmControllerStatus.Init
            elif status == "ready":
                self._status_machine = ArmControllerStatus.Ready
            elif status == "running":
                self._status_machine = ArmControllerStatus.Running
            elif status == "hold_position":
                self._status_machine = ArmControllerStatus.HoldPosition
            elif status == "stopped":
                self._status_machine = ArmControllerStatus.Stopped
            elif status == "error":
                self._status_machine = ArmControllerStatus.Error
    
    def set_view(self, view:bool):
        with self._status_lock:
            self._view = view
    # ==================== getting ====================
    def get_status(self):
        pass
    
    def get_thread_is_alive(self):
        pass
    
    def get_current_status(self):
        with self._status_lock:
            if self._status_machine == ArmControllerStatus.Init:
                return "init"
            elif self._status_machine == ArmControllerStatus.Ready:
                return "ready"
            elif self._status_machine == ArmControllerStatus.Running:
                return "running"
            elif self._status_machine == ArmControllerStatus.HoldPosition:
                return "hold_position"
            elif self._status_machine == ArmControllerStatus.Stopped:
                return "stopped"
            elif self._status_machine == ArmControllerStatus.Error:
                return "error"
            elif self._status_machine == ArmControllerStatus.Exit:
                return "exit"
            else:
                return "unknown"
    
    # ===================== judge =======================
    def judge_current_status(self,status:str) -> bool:
        with self._status_lock:
            if status == "init":
                return True
            elif status == "ready":
                return True
            elif status == "running":
                return True
            elif status == "hold_position":
                return True
            elif status == "stopped":
                return True
            elif status == "error":
                return True
            elif status == "exit":
                return True
            else:
                return False
    
    def judge_cmd(self, cmd:str) -> bool:
        with self._data_lock:
            if self._current_cmd == cmd:
                return True
            else:
                return False
    
    # ==================== send =======================
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
        # task_check_cnt = 0
        # print(f"[Device {self._device_id}] task loop start time: {self._start_time - time.time()}")
        if self.device.get_session_holder() != self.device.get_my_session_id():
            print(f"[Device {self._device_id}] myssid:{self.device.get_my_session_id()} holdssid:{self.device.get_session_holder()}")
        
        while not self.judge_current_status("ready"):
            time.sleep(0.1)
        self.set_status("running")
        
        
        while True: # what is the condition: hex_device_api is running
            try:
                
                # =========== get target position ===========
                target_position = None
                if self.trajectory_player:
                    target_position = self.trajectory_player.get_current_target().tolist()
                
                # ============ send command to device ============
                # if self.judge_current_status("running") and target_position is not None:
                    # self.device.motor_command(
                    #     CommandType.POSITION,
                    #     target_position)
                
                    # self.send_view_data(target_position)
                
                self.device.motor_command(
                    CommandType.POSITION,
                    target_position)
                
                # ============== judge status machine ==============
            
                if self.judge_current_status("init"):
                    pass
                elif self.judge_current_status("ready"):
                    
                    """
                    从 ready 到 running 的条件：
                    - 接到home命令，或者 play命令
                    
                    ready 状态下的行为：
                    - 等待命令
                    - 状态更新
                    """
                    pass
                elif self.judge_current_status("running"):
                    """
                    从 running 到 hold_position 的条件：
                    - 接到 hold_position 命令
                    从 running 到 ready 的条件：
                    - 完成home
                    - 完成单次轨迹
                    从 running 到 stopped 的条件：
                    - 接到 stop 命令
                    从 running 到 error 的条件：
                    - 设备异常
                    
                    running 状态下的行为：
                    - 持续执行轨迹
                    - 状态更新
                    """
                    pass
                elif self.judge_current_status("hold_position"):
                    """
                    从 hold_position 到 running 的条件：
                    - 接到 恢复 命令
                    
                    
                    
                    """
                    pass
                elif self.judge_current_status("stopped"):
                    """
                    从 stopped 到 exit：
                    - home 完成
                    
                    stoped 状态下的行为：
                    - 等待 home 完成
                    
                    """
                    
                    pass
                elif self.judge_current_status("error"):
                    """
                    从 error 到 stop：
                    - 接到 stop 命令
                    
                    """
                    pass
                
                
                
                # ============= print logger ===========

                
                # ============= update =============
                time.sleep(task_loop_hz)
                self.send_view_data(target_position)
                    
                    
                    
            except Exception as e:
                print(f"[Device {self._device_id}] threading Exception: {e}")
                traceback.print_exc()
            except BrokenBarrierError as e:
                print(f"[Device {self._device_id}] Waring Barrier broken")
            # finally:
            #     print(f"[Device {self._device_id}] finally:  threading exit..")
            