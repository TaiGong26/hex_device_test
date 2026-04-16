import time
from typing import List, Optional, Dict
import numpy as np
import traceback
import copy
from enum import Enum
from collections import deque
import multiprocessing as mp
# mp.set_start_method('forkserver', force=True)

from hex_device import HexDeviceApi
from hex_device import Arm, CommandType, Hands

from hex_device_test.controllers.ErrorChecker import ArmErrorChecker
from ..tools.plotjuggle import PlotjuggleDraw
from ..tools.CsvLogger import CsvLogger

from .BaseController import BaseController
from .TrajectoryController import TrajectoryPlanner

from ..statuses.ArmProcessIPC import ArmCommChannel
from ..statuses.ArmStatus import ArmControllerStatus,ArmErrorStatus
from ..controllers.arm_state_machine_process import ArmControllerProcessStateMachine

# class ArmStatusTracker:
#     """
#     设备状态表管理
#     - 运行时间
#     - 运行时错误（dict）
#     - 电机温度
#     """
    
#     def __init__(self, device_id: int):
#         self.device_id = device_id
#         self._runtime_start = time.time()
#         self._error_history: Dict[str, str] = {}
#         self._max_temps = [0.0] * 7
    
#     def update(self, device: Arm) -> None:
#         """更新状态表"""
#         # 更新运行时间
#         runtime = time.time() - self._runtime_start
        
#         # 更新电机温度
#         temps = device.get_temperatures() if hasattr(device, 'get_temperatures') else [0.0] * 6
#         for i, temp in enumerate(temps[:6]):
#             self._max_temps[i] = max(self._max_temps[i], temp)
    
#     def record_error(self, error_code: str, detail: str) -> None:
#         """记录运行时错误"""
#         timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
#         self._error_history[timestamp] = f"[{error_code}] {detail}"
    
#     def get_summary(self) -> Dict:
#         """生成状态摘要"""
#         return {
#             'device_id': self.device_id,
#             'max_temperatures': self._max_temps.copy(),
#             'error_count': len(self._error_history),
#             'errors': self._error_history.copy()
#         }

class ArmStatusTable:
    def __init__(self):
        self._start_time = time.process_time()
        self._motor_temps = [0.0] * 7
        self._log:List[Dict] = []

    
    def update(self):
        pass
    
    def get_summary(self):
        pass

class ArmControllerMp(BaseController):
        
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, task_loop_hz:int=500, device_id:int=0):
        super().__init__(ws_url, local_port, enable_kcp, task_loop_hz, device_id)
        
        # device
        self._arm_config = None
        
        #data 
        # self.data_queue = deque(maxlen=50)
        # self.status_queue = None
        # # lock
        # self._status_lock = mp.Lock()
        # self._data_lock = mp.Lock()

        # Task
        self._loop_running = mp.Event()
        self._task_process = None
        
        # 共享内存引用（由协调器传入）
        self._arm_ipc: Optional[ArmCommChannel] = None
    
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
                self._arm_ipc
            ))
            self._task_process.start()

            return True
    
        except Exception as e:
            print(f"[Device {self._device_id}, Exception]: {e}")
            traceback.print_exc()
            return False
    
    def shutdown(self):
        try:
            
            self._loop_running.value = False
            self._task_process.join(timeout=10)
            # self._task_process.join()
            # 判断进程是否存活，如果是强行杀掉
            if self._task_process.is_alive():
                self._task_process.terminate()
                self._task_process.join()
            
              
            # print(f"[Controller {self._device_id}]: ------------------------------------ shutdown complete ------------------------------------")
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
    
    def set_arm_ipc(self,ipc):
        self._arm_ipc = ipc
    
    
    # ==================== getting ====================
    
    # ===================== judge =======================
    
    # ==================== send =======================

    
    def publish_command(self,command_type:str,target_position):
        pass
    
    # ==================== task ====================
    
    @staticmethod
    def send_view_data(
        sender:PlotjuggleDraw,
        device_id:int, 
        current_position, 
        target_position:Optional[list],
        status_mathine:Optional[Enum], 
        ssid:Optional[int],
        holder:Optional[int],
        state:int,
        error_code:int
        ):
        
        device_key = f"dev{device_id}"
        data = {}

        # 当前值
        if current_position is not None and hasattr(current_position,"tolist"):
            current_position = current_position.tolist()

        # 展开 motor_position
        if current_position is not None:
            for i, v in enumerate(current_position):
                data[f"{device_key}/motor_position/joint{i}"] = float(v)

        # 展开 target_position
        if target_position is not None:
            for i, v in enumerate(target_position):
                data[f"{device_key}/target_position/joint{i}"] = float(v)

        # 标量
        if status_mathine is not None:
            data[f"{device_key}/status"] = status_mathine
        if ssid is not None:
            data[f"{device_key}/ssid"] = ssid
        if holder is not None:
            data[f"{device_key}/holder"] = holder
            
        if state is not None:
            data[f"{device_key}/state"] = state
            
        if error_code is not None:
            data[f"{device_key}/error_code"] = error_code

        sender.add_data(data)
        # sender.send_json(data)
    

    def _task_loop(self, 
        ws_url, 
        enable_kcp, 
        device_id, 
        view,
        loop_running, 
        task_hz, 
        waypoints, 
        arm_ipc:ArmCommChannel):
        """
        子进程主循环
        """
        # 初始化组件
        state_machine = ArmControllerProcessStateMachine(device_id,arm_ipc)
        # status_tracker = ArmStatusTracker(device_id)
        # csv_logger = CsvLogger(device_id)
        sender = PlotjuggleDraw()
        sender.start()
        
        # 初始化设备
        hex_api = HexDeviceApi(ws_url=ws_url, local_port=0, enable_kcp=enable_kcp)
        device = None
        trajectory = None
        
        if waypoints:
            trajectory = TrajectoryPlanner(waypoints=waypoints, segment_duration=3.0)
            # trajectory.start_trajectory()
            
        # Home
        
        task_interval = 1.0 / task_hz
        
        is_view = view
        target_pos = None
        motor_pos = None
        
        try:
            while not loop_running.is_set():
                try:
                    
                    if device is None:
                        for dev in hex_api.device_list:
                            if isinstance(dev, Arm):
                                device = dev
                                break
                    
                    if device is None:
                        time.sleep(task_interval)
                        continue
                    
                    # API退出检查
                    if hex_api.is_api_exit():
                        break
                    
                    # 扫描error
                    # has_error, errors= ArmErrorChecker.check_device(device)
                    # if has_error:
                    #     error_codes = [err_tuple[0] for err_tuple in errors]
                    #     min_error_code = min(error_codes, key=lambda x: x.value)
                    #     arm_ipc.error_status.value = min_error_code.value
                        
                    #     error_details = []
                    #     for code, reasons in errors:
                    #         if isinstance(reasons, list):
                    #             error_details.append(f"{code.name}: {', '.join(map(str, reasons))}")
                    #         else:
                    #             error_details.append(f"{code.name}: {reasons}")
                    #     final_error_msg = " | ".join(error_details)
                        
                    #     state_machine.transition(ArmControllerStatus.Brake, f"errors: {final_error_msg}")
                        
                    # 获取当前目标位置
                    target_pos = trajectory.get_current_target()
                    motor_pos = device.get_motor_positions() 
                    last_pos = trajectory.get_last_position()
                    
                    # 判断pipe是否有命令可读，然后更新命令
                    if arm_ipc.cmd_recv_pipe.poll(timeout=0.01):
                        value = arm_ipc.cmd_recv_pipe.recv()
                        arm_ipc.set_cmd_status(value)
                    
                    # 状态机处理
                    current_state = state_machine.get_state()
                    
                    if current_state == ArmControllerStatus.Init:
                        state_machine.handle_init(device)
                        
                    elif current_state == ArmControllerStatus.Ready:
                        state_machine.handle_ready()
                        trajectory.start_trajectory()
                        
                    elif current_state == ArmControllerStatus.Running:
                        state_machine.handle_running(device, target_pos)
                        
                    elif current_state == ArmControllerStatus.Stopped:
                        state_machine.handle_stopped(device,last_pos)
                        
                    elif current_state == ArmControllerStatus.Brake:
                        state_machine.handle_brake(device)
                        
                    elif current_state == ArmControllerStatus.Exit:
                        state_machine.handle_exit()
                        break
                    
                    if is_view:
                        ArmControllerMp.send_view_data(
                                sender,
                                device_id,
                                motor_pos,
                                target_pos,
                                None,
                                dev.get_my_session_id(),
                                dev.get_session_holder(),
                                current_state.value,
                                arm_ipc.get_error_status()
                            )
                    
                    # CSV记录: 30s记录一次
                    # csv_logger.log(current_state, motor_pos, target_pos, temps, error_status)
                    
                    
                    # time.sleep(task_interval)
                    time.sleep(task_interval)
                except KeyboardInterrupt:
                    # break
                    pass
                
                except Exception as e:
                    print(f"[Device {device_id}] 循环异常: {e}")
                    # 记录错误并尝试继续
                    arm_ipc.set_error_status(ArmErrorStatus.ProcessError.value)
                    traceback.print_exc()
                    
        finally:
            pass
        # ================== 资源回收 ===================
        # IPC
        if arm_ipc.cmd_recv_pipe.poll(timeout=0.01):
            value = arm_ipc.cmd_recv_pipe.recv()
        arm_ipc.cmd_recv_pipe.close()
        # csv_logger.close()
        
        # 生成最终报告
        # summary = status_tracker.get_summary()
        # print(f"\n[Device {device_id}] 运行报告:")
        # # print(f"  运行时间: {summary['runtime_seconds']:.2f}秒")
        # print(f"  最高温度: {summary['max_temperatures']}")
        # print(f"  错误次数: {summary['error_count']}")
        
        hex_api.close()