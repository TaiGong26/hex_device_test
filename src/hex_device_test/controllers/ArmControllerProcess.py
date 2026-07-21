import time
from typing import List, Optional, Dict
import numpy as np
import traceback
import copy
from enum import Enum
from collections import deque
import multiprocessing as mp
import csv
import os
# mp.set_start_method('forkserver', force=True)

from hex_device import Hands, HexDeviceApi
from hex_device import Arm

from hex_device_test.controllers.ErrorChecker import ArmErrorChecker
from ..tools.plotjuggle import PlotjuggleDraw

from .BaseController import BaseController
from .TrajectoryController import TrajectoryPlanner, SegmentedTrajectoryPlanner
from ..tools.trajectory_loader import DEFAULT_SEGMENT_DURATION

from ..statuses.ArmProcessIPC import ArmCommChannel
from ..statuses.ArmStatus import ArmControllerStatus,ArmErrorStatus
from ..controllers.arm_state_machine_process import ArmControllerProcessStateMachine
from datetime import timedelta

class ArmStatusTable:
    def __init__(self):
        self._start_time = time.time()
        self._state:ArmErrorStatus = ArmErrorStatus.Normal
        self._run_time = 0.0
        self._motor_max_temps = None
        self._driver_max_temps = None
        self._hands_motor_max_temps = None
        self._hands_driver_max_temps = None
        self._errors = deque(maxlen=10)
        self._error_set = set()

    def update(self, motor_temps:Optional[np.ndarray], driver_temps:Optional[np.ndarray],
               hands_motor_temps:Optional[np.ndarray] = None,
               hands_driver_temps:Optional[np.ndarray] = None):
        self._run_time = time.time() - self._start_time

        if motor_temps is not None:
            motor_temps = np.asarray(motor_temps, dtype=float)
            if self._motor_max_temps is None:
                self._motor_max_temps = motor_temps.copy()
            else:
                np.maximum(self._motor_max_temps, motor_temps, out=self._motor_max_temps)

        if driver_temps is not None:
            driver_temps = np.asarray(driver_temps, dtype=float)
            if self._driver_max_temps is None:
                self._driver_max_temps = driver_temps.copy()
            else:
                np.maximum(self._driver_max_temps, driver_temps, out=self._driver_max_temps)

        if hands_motor_temps is not None:
            hands_motor_temps = np.asarray(hands_motor_temps, dtype=float)
            if self._hands_motor_max_temps is None:
                self._hands_motor_max_temps = hands_motor_temps.copy()
            else:
                np.maximum(self._hands_motor_max_temps, hands_motor_temps, out=self._hands_motor_max_temps)

        if hands_driver_temps is not None:
            hands_driver_temps = np.asarray(hands_driver_temps, dtype=float)
            if self._hands_driver_max_temps is None:
                self._hands_driver_max_temps = hands_driver_temps.copy()
            else:
                np.maximum(self._hands_driver_max_temps, hands_driver_temps, out=self._hands_driver_max_temps)

    def set_error(self, state, error_msg: str):
        key = (state,error_msg)
        if key in self._error_set:
            return
        self._error_set.add(key)
        
        self._state:ArmErrorStatus = state
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self._errors.append(f"[{timestamp}] errType: {state.name}, errMsg: {error_msg}")

    def get_summary(self):
        motor_temps = self._motor_max_temps.tolist() if hasattr(self._motor_max_temps,"tolist") else None
        dev_temps =  self._driver_max_temps.tolist() if hasattr( self._driver_max_temps,"tolist") else None
        hands_motor_temps = self._hands_motor_max_temps.tolist() if hasattr(self._hands_motor_max_temps,"tolist") else None
        hands_dev_temps =  self._hands_driver_max_temps.tolist() if hasattr( self._hands_driver_max_temps,"tolist") else None

        return {
            "state": self._state.name,
            "run_time": str(timedelta(seconds=int(self._run_time))),
            "motor_max_temperature": motor_temps,
            "motor_driver_max_temperature": dev_temps,
            "hands_motor_max_temperature": hands_motor_temps,
            "hands_driver_max_temperature": hands_dev_temps,
            "errors": self._errors.copy()
        }

class ArmControllerMp(BaseController):
        
    def __init__(self, ws_url:str, local_port:int=0, enable_kcp:bool=False, task_loop_hz:int=500, device_id:int=0):
        super().__init__(ws_url, local_port, enable_kcp, task_loop_hz, device_id)
        
        # device
        self._arm_config = None
        self._waypoints = None
        self._segment_duration = None
        self._segment_ends = None
        self._time_sleep = None
        self._interpolate = True

        # Task
        self._loop_running = mp.Event()
        self._task_process = None
        
        # 共享内存引用（由协调器传入）
        self._arm_ipc: Optional[ArmCommChannel] = None
        self._mp_queue = None
        self._temp_csv_dir = None
    
    def start(self) -> bool:
        
        try: 
            
            self._task_process = mp.Process(target=self._task_loop, args=(
                self._ws_url,
                self._enable_kcp,
                self._device_id,
                self._view,
                self._check_timeout,
                self._loop_running,
                self._task_loop_hz,
                self._waypoints,
                self._segment_duration,
                self._segment_ends,
                self._time_sleep,
                self._interpolate,
                self._arm_ipc,
                self._arm_config,
                self._mp_queue,
                self._temp_csv_dir
            ))
            self._task_process.start()

            return True
    
        except Exception as e:
            print(f"[Device {self._device_id}, Exception]: {e}")
            traceback.print_exc()
            return False
    
    def shutdown(self):
        try:
            
            self._loop_running.set()
            self._task_process.join(timeout=5)
            
            if self._task_process.is_alive():
                self._task_process.terminate()
                self._task_process.join()
              
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

    def set_segment_duration(self, segment_duration):
        self._segment_duration = segment_duration

    def set_segment_ends(self, segment_ends):
        self._segment_ends = segment_ends

    def set_time_sleep(self, time_sleep):
        self._time_sleep = time_sleep

    def set_interpolate(self, interpolate: bool):
        self._interpolate = interpolate
    
    def set_view(self, view):
        self._view = view
        
    def set_check_timeout(self, check):
        self._check_timeout = check
    
    def set_arm_ipc(self,ipc):
        self._arm_ipc = ipc
        
    def set_mp_queue(self,queue):
        self._mp_queue = queue
    
    def set_temp_csv_dir(self, temp_csv_dir: str):
        self._temp_csv_dir = temp_csv_dir
    
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

        if current_position is not None and hasattr(current_position,"tolist"):
            current_position = current_position.tolist()

        if current_position is not None:
            for i, v in enumerate(current_position):
                data[f"{device_key}/motor_position/joint{i}"] = float(v)

        if target_position is not None:
            for i, v in enumerate(target_position):
                data[f"{device_key}/target_position/joint{i}"] = float(v)

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
    
    def _task_loop(self, 
        ws_url, 
        enable_kcp, 
        device_id, 
        view,
        check_timeout,
        loop_running, 
        task_hz, 
        waypoints,
        segment_duration,
        segment_ends,
        time_sleep,
        interpolate,
        arm_ipc:ArmCommChannel,
        arm_config,
        mp_queue:mp.Queue,
        temp_csv_dir:Optional[str] = None):
        """
        子进程主循环
        """
        # 初始化组件
        state_machine = ArmControllerProcessStateMachine(device_id,arm_ipc)
        device_state = ArmStatusTable()
        sender = PlotjuggleDraw()
        sender.start()
        
        # 初始化设备
        hex_api = HexDeviceApi(ws_url=ws_url, local_port=0, enable_kcp=enable_kcp)
        device = None
        hands:Optional[Hands] = None
        
        trajectory = None
        
        if waypoints:
            duration = segment_duration if segment_duration is not None else DEFAULT_SEGMENT_DURATION
            if segment_ends:
                trajectory = SegmentedTrajectoryPlanner(
                    waypoints=waypoints,
                    segment_ends=segment_ends,
                    segment_duration=duration,
                    hold_duration=time_sleep if time_sleep is not None else 0.0,
                    interpolate=interpolate,
                )
                print(
                    f"dev{device_id}: segmented trajectory, {len(segment_ends)} segment(s), "
                    f"segment_duration={duration}s, time_sleep={time_sleep}s, "
                    f"interpolate={interpolate}, "
                    f"segment_ends={segment_ends}"
                )
            else:
                trajectory = TrajectoryPlanner(
                    waypoints=waypoints,
                    segment_duration=duration,
                    interpolate=interpolate,
                )
                print(
                    f"dev{device_id}: trajectory segment_duration={duration}s, "
                    f"interpolate={interpolate}"
                )
            
        task_interval = 1.0 / task_hz
        
        # 温度 CSV 文件（每个臂一个文件）
        temp_csv_file = None
        temp_csv_writer = None
        last_temp_log_time = 0.0
        if temp_csv_dir:
            os.makedirs(temp_csv_dir, exist_ok=True)
            start_ts = time.strftime("%Y-%m-%d_%H-%M-%S")
            csv_path = os.path.join(temp_csv_dir, f"temp_dev{device_id}_{start_ts}.csv")
            temp_csv_file = open(csv_path, 'w', newline='')
            temp_csv_writer = csv.writer(temp_csv_file)
            # header: time + motor0~motor5
            header = ["timestamp"] + [f"motor_{i}" for i in range(6)]
            temp_csv_writer.writerow(header)
            print(f"dev{device_id}: temperature CSV -> {csv_path}")
        
        is_view = view
        target_pos = None
        motor_pos = None
        _timeout = 0.0
        loop_counter = 0
        prev_segment_index = None
        
        try:
            while not loop_running.is_set():
                try:
                    
                    if device is None:
                        for dev in hex_api.device_list:
                            if isinstance(dev, Arm):
                                device = dev
                                if not device.reload_arm_config_from_dict(arm_config):
                                    state_machine.transition(ArmControllerStatus.Exit, f"errors: device{device_id} not arm config")
                                print(f"dev{device_id}: robot_type{device.robot_type}")
                                break
                    
                    if hands is None:
                        for hand in hex_api.optional_device_list:
                            if isinstance(device, Hands):
                                hands = hand
                                if hands.has_new_data():
                                    if hands_first_time:
                                        hands_first_time = False
                                        
                                        
                                        
                    if device is None:
                        time.sleep(task_interval)
                        _timeout+=task_interval
                        continue
                    
                    # API退出检查
                    if hex_api.is_api_exit():
                        break
                    
                    connLost = hex_api.is_websocket_recv_timeout()
                    
                    # 扫描error
                    has_error, errors= ArmErrorChecker.check_device(check_timeout,device,connLost, hands)
                    if has_error:
                        error_codes:ArmErrorStatus = [err_tuple[0] for err_tuple in errors]
                        min_error_code = min(error_codes, key=lambda x: x.value)
                        arm_ipc.set_error_status(min_error_code.value)
                        
                        # 处理电机错误或者Arm错误
                        error_details = []
                        err_c = None
                        for code, reasons in errors:
                            err_c = code
                            info = ArmErrorChecker.format_error(err_c,reasons)
                            error_details.append(f"{code.name}: {info}")
                            device_state.set_error(code,info)
                                
                        final_error_msg = " | ".join(error_details)
                        
                        state_machine.transition(ArmControllerStatus.Brake, f"errors: {final_error_msg}")
                        
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
                        device._last_command_time = None # 根据sdk，在last_command==None时候可以发送空命令
                        
                        
                    elif current_state == ArmControllerStatus.Running:
                        state_machine.handle_running(device, target_pos, hands)
                        segment_info = trajectory.get_current_segment_info()
                        
                        # check loop
                        if int(segment_info['total_elapsed'] * 10) % 5 == 0:  # Print every 0.5 seconds
                            if prev_segment_index is not None:
                                if segment_info['segment_index'] == 0 and prev_segment_index != 0:
                                    loop_counter += 1

                            prev_segment_index = segment_info['segment_index']
                        
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
                    
                    # running state update
                    motor_temps = device.get_motor_temperatures()
                    driver_temps = device.get_motor_driver_temperatures()
                    hands_motor_temps = hands.get_motor_temperatures() if hands is not None else None
                    hands_driver_temps = hands.get_motor_driver_temperatures() if hands is not None else None
                    device_state.update(motor_temps, driver_temps, hands_motor_temps, hands_driver_temps)
                    
                    # 每 ~1 秒记录一次电机温度到 CSV（基于实际时间间隔）
                    if temp_csv_writer is not None:
                        now = time.monotonic()
                        if now - last_temp_log_time >= 1.0:
                            last_temp_log_time = now
                            ts = time.strftime("%Y-%m-%d %H:%M:%S")
                            row = [ts]
                            if motor_temps is not None:
                                row.extend(float(v) for v in motor_temps[:6])
                            else:
                                row.extend([""] * 6)
                            temp_csv_writer.writerow(row)
                            temp_csv_file.flush()
                    
                    time.sleep(task_interval)
                
                except Exception as e:
                    print(f"[Dev {device_id}] 循环异常: {e}")
                    arm_ipc.set_error_status(ArmErrorStatus.ProcessError.value)
                
                except KeyboardInterrupt:
                    pass
        finally:
            if temp_csv_file:
                temp_csv_file.close()
            pass
        # ================== 资源回收 ===================
        # IPC close
        if arm_ipc.cmd_recv_pipe.poll(timeout=0.01):
            value = arm_ipc.cmd_recv_pipe.recv()
        arm_ipc.cmd_recv_pipe.close()
        
        # report
        report = {}
        report[device_id] = device_state.get_summary()
        report[device_id].update({
            "loop_counter":loop_counter,
            
        })
        mp_queue.put(report)
        
        hex_api.close()