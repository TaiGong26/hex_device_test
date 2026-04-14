import time
from typing import Optional, Dict
import numpy as np
import traceback
import copy
from enum import Enum
from collections import deque
import multiprocessing as mp


from hex_device import HexDeviceApi
from hex_device import Arm, CommandType, Hands
from ..tools.plotjuggle import PlotjuggleDraw
from ..tools.CsvLogger import CsvLogger

from .BaseController import BaseController
from .TrajectoryController import TrajectoryPlanner

from ..statuses.ArmProcessCommunication import ArmCommChannel
from ..statuses.ArmStatus import ArmControllerProcessStateMachine,ArmControllerStatus,ArmErrorStatus


class ArmStatusTracker:
    """
    设备状态表管理
    - 运行时间
    - 运行时错误（dict）
    - 电机温度
    """
    
    def __init__(self, device_id: int, arm_ipc: ArmCommChannel):
        self.device_id = device_id
        self._ipc = arm_ipc
        self._runtime_start = time.time()
        self._error_history: Dict[str, str] = {}
        self._max_temps = [0.0] * 6
    
    def update(self, device: Arm) -> None:
        """更新状态表"""
        # 更新运行时间
        runtime = time.time() - self._runtime_start
        self._ipc.runtime_seconds.value = runtime
        
        # 更新电机温度
        temps = device.get_temperatures() if hasattr(device, 'get_temperatures') else [0.0] * 6
        for i, temp in enumerate(temps[:6]):
            self._ipc.motor_temps[i] = temp
            self._max_temps[i] = max(self._max_temps[i], temp)
    
    def record_error(self, error_code: str, detail: str) -> None:
        """记录运行时错误"""
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self._error_history[timestamp] = f"[{error_code}] {detail}"
    
    def get_summary(self) -> Dict:
        """生成状态摘要"""
        return {
            'device_id': self.device_id,
            'runtime_seconds': self._ipc.runtime_seconds.value,
            'max_temperatures': self._max_temps.copy(),
            'error_count': len(self._error_history),
            'errors': self._error_history.copy()
        }

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
        
        
        # 共享内存引用（由协调器传入）
        self._arm_ipc: Optional[ArmCommChannel] = None
        
        # 状态机
        # self._state_machine: Optional[ArmControllerProcessStateMachine] = None
        
        # CSV日志
        # self._csv_logger: Optional[CsvLogger] = None
        
        # 设备状态表
        # self._status_tracker: Optional[ArmStatusTracker] = None
        
        
        
        # trajectory And Home
        # self._waypoints = None
        # self.__HOME_POSITION = tuple([0.0, -1.5, 3.00, 0.0, 0.0, 0.0])
        # self._return_home_duration = 10
        
    
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
    
    def set_arm_ipc(self,ipc):
        self._arm_ipc = ipc
    
    
    # ==================== getting ====================
    
    # ===================== judge =======================
    
    # ==================== send =======================
    def send_view_data(
        self,
        sender:PlotjuggleDraw,
        device_id:int, 
        current_position, 
        target_position:Optional[list],
        status_mathine:Optional[Enum], 
        ssid:Optional[int],
        holder:Optional[int]
        ):
        
        device_key = f"dev{device_id}"
        data = {}

        # 当前值
        if current_position is not None and hasattr(current_position,"tolist"):
            current_position = current_position.tolist()

        # 展开 motor_position
        if current_position:
            for i, v in enumerate(current_position):
                data[f"{device_key}/motor_position/joint{i}"] = float(v)

        # 展开 target_position
        if target_position:
            for i, v in enumerate(target_position):
                data[f"{device_key}/target_position/joint{i}"] = float(v)

        # 标量
        if status_mathine is not None:
            data[f"{device_key}/status"] = status_mathine
        if ssid is not None:
            data[f"{device_key}/ssid"] = ssid
        if holder is not None:
            data[f"{device_key}/holder"] = holder

        sender.send_json(data)
    
    def publish_command(self,command_type:str,target_position):
        pass
    
    
    
    # ==================== task ====================

    # def _task_loop(self,
    #     ws_url,
    #     enable_kcp,
    #     device_id,
    #     view,
    #     loop_running,
    #     task_hz,
    #     waypoints,
    #     ):
        
    #     # 声明
    #     task_loop_hz = 1 / task_hz
    #     target_position = None
    #     _waypoints = waypoints
    #     first_time = True
    #     _loop_running = loop_running
    #     is_view = view
        
    #     # object
    #     device:Arm = None
    #     sender = PlotjuggleDraw()
    #     trajectory_player:Optional[TrajectoryPlanner] = None
    #     return_Home = None
        
    #     # 初始化
    #     hex_api:HexDeviceApi = HexDeviceApi(
    #         ws_url=ws_url, 
    #         local_port=0, 
    #         enable_kcp=enable_kcp, 
    #     )
    #     start_time = time.process_time()
        
    #     if _waypoints:
    #         trajectory_player = TrajectoryPlanner(
    #             waypoints=_waypoints,
    #             segment_duration=3.0
    #             )
    #     else:
    #         print(f"[dev{device_id}] Not waypoints !!!!")
    #         return False
        
    #     if trajectory_player:
    #         res = trajectory_player.start_trajectory()
    #         if res:
    #             print(f"[dev{device_id}] trajectory player start ")
    #         else:
    #             print(f"[dev{device_id}] Not waypoints failed !!!!")

    #     while _loop_running.is_set() == False: # what is the condition: hex_device_api is running
    #         try:
    #             if hex_api.is_api_exit():
    #                 break
                
    #             if trajectory_player:
    #                 target_position = trajectory_player.get_current_target().tolist()
                
    #             for dev in hex_api.device_list:
    #                 if isinstance(dev,Arm):
                        
    #                     if first_time:
    #                         first_time = False
    #                         dev.start()
                        
    #                     send_data = {}
    #                     device_key = f"dev{device_id}"
                        
    #                     # command
    #                     if target_position is not None and isinstance(target_position,list):
    #                         dev.motor_command(CommandType.POSITION,target_position)
    #                         # send_data[f"{device_key}/target_position"] = target_position.copy()
    #                     else:
    #                         print(f"[dev{device_id}] target None {target_position}")
                            
    #                     # # send_view
    #                     # dev_position = dev.get_motor_positions(pop=False)
    #                     # if dev_position is not None:
    #                     #     for i, v in enumerate(dev_position):
    #                     #         send_data[f"{device_key}/motor_position/joint{i}"] = float(v)
    #                     #     # print(f"[dev{device_id}] Position: {dev_position}")
    #                     # # 展开 target_position
    #                     # if target_position is not None:
    #                     #     for i, v in enumerate(target_position):
    #                     #         send_data[f"{device_key}/target_position/joint{i}"] = float(v)
    #                     #     # print(f"[dev{device_id}] target: {target_position}")
                        
    #                     # send_data[f"{device_key}/ssid"] = dev.get_my_session_id()
    #                     # send_data[f"{device_key}/hold_ssid"] = dev.get_session_holder()
    #                     # sender.send_json(send_data)
                        
    #                     dev_position = dev.get_motor_positions(pop=False)
                        
    #                     self.send_view_data(
    #                         sender,
    #                         device_id,
    #                         dev_position,
    #                         target_position,
    #                         None,
    #                         dev.get_my_session_id(),
    #                         dev.get_session_holder()
    #                     )
                        
                        
                    
    #             # ============== status machine ==============
                
    #             # ============== check error ================

    #             # ============= update =============
    #             time.sleep(task_loop_hz)
    #             # send_view_data(target_position)
                    
                    
    #         except Exception as e:
    #             print(f"[dev {device_id}] process Exception: {e}")
    #             # traceback.print_exc()
    #         except KeyboardInterrupt:
    #             break
    #         finally:
    #             pass
        
    #     hex_api.close()
    #     # print(f"[dev{device_id}]: close Process")
        
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
        state_machine = ArmControllerProcessStateMachine(arm_ipc)
        status_tracker = ArmStatusTracker(device_id, arm_ipc)
        csv_logger = CsvLogger(device_id)
        sender = PlotjuggleDraw()
        
        
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
                    
                    # 更新状态表
                    status_tracker.update(device)
                    
                    # 获取当前目标位置
                    target_pos = trajectory.get_current_target()
                    motor_pos = device.get_motor_positions() 
                    last_pos = trajectory.get_last_position()
                    # temps = [arm_ipc.motor_temps[i] for i in range(6)]
                    
                    # 判断pipe是否有命令可读，然后更新命令
                    if arm_ipc.cmd_recv_pipe.poll(timeout=0.01):
                        value = arm_ipc.cmd_recv_pipe.recv()
                        arm_ipc.cmd_status.value = value
                    
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
                        self.send_view_data(
                                sender,
                                device_id,
                                motor_pos,
                                target_pos,
                                None,
                                dev.get_my_session_id(),
                                dev.get_session_holder()
                            )
                    
                    # CSV记录
                    # error_status = ArmErrorStatus(arm_ipc.error_status.value)
                    # csv_logger.log(current_state, motor_pos, target_pos, temps, error_status)
                    
                    time.sleep(task_interval)
                    
                except Exception as e:
                    print(f"[Device {device_id}] 循环异常: {e}")
                    # 记录错误并尝试继续
                    arm_ipc.error_status.value = ArmErrorStatus.Error.value
                    
        finally:
            pass
        # 清理
        csv_logger.close()
        
        # 生成最终报告
        summary = status_tracker.get_summary()
        print(f"\n[Device {device_id}] 运行报告:")
        print(f"  运行时间: {summary['runtime_seconds']:.2f}秒")
        print(f"  最高温度: {summary['max_temperatures']}")
        print(f"  错误次数: {summary['error_count']}")
        
        hex_api.close()