"""
ArmControllerMpV2 — 重构版控制器（Worker 模式）

设计意图：
将 ArmControllerProcess._task_loop（231 行）拆分为 6 个结构化方法，通过
run() 入口统一调度。IPC 保持 Pipe + mp.Value 不变（只修 poll 超时）。

run()
  ├── init_var()      —— 变量初始化
  ├── init_mod()      —— 模块创建（状态机 / API / 轨迹 / CSV / 可视化）
  ├── while loop:
  │   ├── state_check()   —— 设备发现 + 错误扫描
  │   ├── state_transition()  —— Pipe 读取 + 状态机 dispatch
  │   └── update_state_to_flie()      —— 轨迹目标 + 温度 + CSV + View
  └── _close()        —— finally 资源回收

与旧版的关键差异：
  - 所有配置通过构造函数传入，无 setter
  - poll(timeout=0) 替代 poll(0.01)，消除 10ms 阻塞
  - daemon=True 移除
  - PlotjuggleDraw 仅在 enable_view=True 时创建
"""

#### TODO， 增加Json读取功能，使得适配不同的arm


import time
import csv
import os
import signal
import logging
import traceback
from typing import Optional, List
import multiprocessing as mp

from hex_device_test.controllers.state_checker import StateChecker
from hex_device_test.devices.arm_wrapper import ArmWrapper
from hex_device_test.devices.wrapper_base import WrapperParams
from ..tools.plotjuggle import PlotjuggleDraw
from .BaseController import BaseController
from .TrajectoryController import (
    DEFAULT_SEGMENT_DURATION,
    TimestampsTrajectoryPlanner,
    TrajectoryPlanner,
)
from ..statuses.ArmProcessIPC import ArmCommChannel
from ..statuses.ArmStatus import ArmControllerStatus, ArmErrorStatus
from ..controllers.arm_state_machine_process import ArmControllerProcessStateMachine
from ..tools.log_config import setup_logging


class ArmControllerMpV2(BaseController):
    """重构版控制器——所有配置通过构造函数注入"""

    def __init__(self, ws_url: str, device_id: int = 0, enable_kcp: bool = False,
                 task_loop_hz: int = 500, arm_ipc: Optional[ArmCommChannel] = None,
                 robot_type: str = "Archer_y6",
                 waypoints: Optional[list] = None,
                 segment_duration: Optional[float] = None,
                 interpolate: str = 'linear',
                 timestamps: Optional[list] = None,
                 mp_queue: Optional[mp.Queue] = None,
                 temp_csv_dir: Optional[str] = None,
                 enable_view: bool = False, 
                #  check_timeout: bool = False,
                 ):
        super().__init__(ws_url, 0, enable_kcp, task_loop_hz, device_id)

        self._logger = logging.getLogger(f"Dev{self._device_id}")

        # 模块/设备配置
        self._arm_ipc = arm_ipc
        self._robot_type = robot_type

        # 轨迹配置
        self._waypoints = waypoints
        self._segment_duration = segment_duration
        self._interpolate = interpolate
        self._timestamps = timestamps

        # 运行配置
        self._mp_queue = mp_queue
        self._temp_csv_dir = temp_csv_dir
        self._enable_view = enable_view
        # self._check_timeout = check_timeout

        # 进程控制
        self._loop_running = mp.Event()
        self._task_process: Optional[mp.Process] = None

    # ==================== 外部接口 ====================

    def start(self) -> bool:
        """启动子进程"""
        try:
            self._task_process = mp.Process(target=self.run)
            self._task_process.start()
            return True
        except Exception as e:
            self._logger.error("start() failed: %s", e)
            traceback.print_exc()
            return False

    def shutdown(self):
        """停止子进程（Event → join → terminate）"""
        try:
            self._loop_running.set()
            self._task_process.join(timeout=5)
            if self._task_process and self._task_process.is_alive():
                self._task_process.terminate()
                self._task_process.join()
        except RuntimeError as e:
            self._logger.error("shutdown RuntimeError: %s", e)
        except Exception as e:
            self._logger.error("shutdown Exception: %s", e)

    # ==================== 子进程入口 ====================

    def run(self):
        """
        子进程入口（mp.Process(target=self.run) 启动）

        init_mod 在 try 内执行，失败则设 _loop_initialized=False，
        跳过主循环直接进入 finally _close()，确保资源回收。
        """
        # 子进程与父进程同属一个进程组：父进程 Ctrl+C 时本进程也会收到 SIGINT。
        # 显式忽略 SIGINT —— 子进程生命周期完全由父进程 shutdown() 控制
        # （Event → join → terminate），避免 init_mod 阻塞建连 / sleep 期间
        # 被 KeyboardInterrupt 中途打断、跳过 finally _close() 资源回收。
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # 子进程入口确保日志配置（fork 下继承父进程 handler，此处幂等 no-op）
        setup_logging()

        self.init_var()

        try:
            self.init_mod()
            self._loop_initialized = True
        except Exception as e:
            self._logger.error("init_mod failed: %s", e)
            traceback.print_exc()
            self._loop_initialized = False

        try:
            if not self._loop_initialized:
                return

            while not self._loop_running.is_set():
                try:

                    self.state_check()
                    
                    self.state_transition()

                    self.update_state_to_flie()
                    
                    time.sleep(self._task_interval)

                except KeyboardInterrupt:
                    # 安全兜底：SIGINT 已在 run() 入口进程级忽略，此处防御程序性抛出的 KeyboardInterrupt
                    pass

                except Exception as e:
                    self._logger.error("循环异常: %s", e)
                    self._arm_ipc.set_error_status(ArmErrorStatus.ProcessError.value)
                    traceback.print_exc()
        finally:
            self._close()


    def init_var(self):
        """变量初始化（只赋初值，无需回收）"""
        self._task_interval = 1.0 / self._task_loop_hz
        self._device: Optional[ArmWrapper] = None
        self._trajectory = None
        self._target_pos = None
        self._motor_pos = None
        self._last_pos = None
        self._loop_counter = 0
        self._prev_segment_index = None
        self._last_temp_log_time = 0.0
        self._loop_initialized = False
        
        # 组件
        self._temp_csv_file = None
        self._temp_csv_writer = None

    def init_mod(self):
        """模块创建（状态机 / API / 轨迹设备发现 + 错误扫描

        分四步：
        1. 设备发现（仅一次）—— 找到第一个 Arm 实例
        2. API 退出检查
        3. 连接丢失检查
        4. ArmErrorChecker 全面错误扫描 → Brake / CSV / 可视化）"""
        # 状态机
        self._state_machine = ArmControllerProcessStateMachine(
            self._device_id, self._arm_ipc, self._waypoints[0]
        )
        # 设备状态表（温度 / 错误记录）
        self._device_state = StateChecker()

        # 可视化（仅开启时创建 PlotjuggleDraw UDP sender）
        self._sender = None
        if self._enable_view:
            self._sender = PlotjuggleDraw()
            self._sender.start()
        
        # 设备接入：ArmWrapper（hex_driver_robot 后端），接收一个 WrapperParams
        # host/port 从 self._ws_url（ws://host:port）解析，透传到底层驱动
        host, port = self._parse_ws_url(self._ws_url)
        param = WrapperParams(
            enable_kcp=self._enable_kcp,
            log_level="ERROR",  # 临时屏蔽 driver DEBUG/INFO 刷屏（原 "DEBUG"）
            grip_type="empty",
            robot_name=self._robot_type,
            host=host,
            port=port,
        )
        self._device = ArmWrapper(params=param)
        
        # 轨迹规划器
        if self._timestamps:
            # 录制轨迹 → 时间戳驱动 + 循环（耐久性）
            self._trajectory = TimestampsTrajectoryPlanner(
                waypoints=self._waypoints,
                timestamps=self._timestamps,
                interpolate=self._interpolate,
                loop=True,
            )
            self._logger.info(
                "recorded trajectory, %d waypoints, "
                "interpolate=%s, loop=True",
                len(self._waypoints), self._interpolate,
            )
        elif self._waypoints:
            # 默认轨迹 → 均匀段时长驱动
            duration = (self._segment_duration
                        if self._segment_duration is not None
                        else DEFAULT_SEGMENT_DURATION)
            self._trajectory = TrajectoryPlanner(
                waypoints=self._waypoints,
                segment_duration=duration,
                interpolate=self._interpolate,
            )
            self._logger.info(
                "trajectory segment_duration=%ss, interpolate=%s",
                duration, self._interpolate,
            )

        # CSV 日志文件（header 推迟到 error_check 发现设备后写入）

        if self._temp_csv_dir:
            os.makedirs(self._temp_csv_dir, exist_ok=True)
            start_ts = time.strftime("%Y-%m-%d_%H-%M-%S")
            csv_path = os.path.join(
                self._temp_csv_dir,
                f"temp_dev{self._device_id}_{start_ts}.csv"
            )
            self._temp_csv_file = open(csv_path, 'w', newline='')
            self._temp_csv_writer = csv.writer(self._temp_csv_file)
            self._logger.info("temperature CSV -> %s", csv_path)


    def state_check(self):
        
        # 计算轨迹位置
        self._target_pos = (self._trajectory.get_current_target()
                            if self._trajectory else None)
        self._motor_pos = self._device.get_motor_positions()
        self._last_pos = (self._trajectory.get_last_position()
                          if self._trajectory else None)
        
        # temperature update
        mt = self._device.get_motor_temperatures()
        dt = self._device.get_motor_driver_temperatures()
        self._device_state.update({"motor_temps": mt, "driver_temps": dt})

        # error check
        arm_err = self._device.get_motor_error_codes()
        
        grip_err = (
            self._device.get_grip_motor_error_codes()
            if self._device.has_grip() else None
        )
        robot_mode = self._device.get_arm_robot_mode()
        self._device_state.update_error(
            arm_err=arm_err,
            grip_err=grip_err,
            robot_mode=robot_mode,
        )

        # 设备错误 → 上报 IPC error_status（coordinator 每 10ms 轮询读取）
        # arm/grip 电机错误 → MotorError；robot_mode 被接管/致命错误 → ArmError
        if self._arm_ipc.get_error_status() == ArmErrorStatus.Normal.value:
            if (arm_err and any(arm_err)) or (grip_err and any(grip_err)):
                self._arm_ipc.set_error_status(ArmErrorStatus.MotorError.value)
                self._logger.warning("error detected -> error_status=MotorError")
            elif robot_mode in ("RmOvertaken", "RmFatalError"):
                self._arm_ipc.set_error_status(ArmErrorStatus.ArmError.value)
                self._logger.warning("error detected -> error_status=ArmError")

    def state_transition(self):
        """
        命令读取 + 状态机 dispatch

        1. 非阻塞读 Pipe（poll(0) 替代 poll(0.01)，消除 10ms 阻塞）
        2. 根据当前状态 dispatch 到对应的 handle_* 方法
        """
        # ── 1. 读取 Pipe 命令 ──
        if self._arm_ipc.cmd_recv_pipe.poll(timeout=0):
            try:
                value = self._arm_ipc.cmd_recv_pipe.recv()
                self._arm_ipc.set_cmd_status(value)
            except EOFError:
                pass

        # ── 2. 状态机 dispatch ──
        current_state = self._state_machine.get_state()

        if current_state == ArmControllerStatus.Init:
            self._state_machine.handle_init(self._device)

        elif current_state == ArmControllerStatus.Ready:
            self._state_machine.handle_ready()
            # Ready 状态下启动轨迹（仅首次）
            if self._trajectory:
                self._trajectory.start_trajectory()

        elif current_state == ArmControllerStatus.Running:
            self._state_machine.handle_running(self._device, self._target_pos)
            # ###TODO: 夹爪回放 —— 第 3 轮范围外，后续在 handle_running 增加
            #           grip_motor_command("position", grip_target)（grip 目标来自规划器）
            if self._trajectory:
                self._update_loop_count()

        elif current_state == ArmControllerStatus.Stopped:
            self._state_machine.handle_stopped(self._device, self._last_pos)

        elif current_state == ArmControllerStatus.Brake:
            self._state_machine.handle_brake(self._device)

        elif current_state == ArmControllerStatus.Exit:
            self._state_machine.handle_exit()
            
    def _update_loop_count(self):
        """分段轨迹循环计数（旧代码 L377-383）"""
        segment_info = self._trajectory.get_current_segment_info()
        if not segment_info:
            return
        if int(segment_info['total_elapsed'] * 10) % 5 == 0:
            if self._prev_segment_index is not None:
                if segment_info['segment_index'] == 0 and self._prev_segment_index != 0:
                    self._loop_counter += 1
            self._prev_segment_index = segment_info['segment_index']

    def update_state_to_flie(self):
        """
        CSV 日志 + 可视化

        """
        # ── 1. 可视化数据 ──
        if self._enable_view and self._sender is not None:
            self._send_view_data()

        # ── 2. CSV 日志（~1Hz） ──
        motor_temps = self._device.get_motor_temperatures()
        if self._temp_csv_writer is not None:
            now = time.monotonic()
            if now - self._last_temp_log_time >= 1.0:
                self._last_temp_log_time = now
                ts = time.strftime("%Y-%m-%d %H:%M:%S")
                row = [ts]
                if motor_temps is not None:
                    n_motors = self._device.motor_count
                    row.extend(float(v) for v in motor_temps[:n_motors])
                else:
                    n_cols = (self._device.motor_count
                              if self._device else 6)
                    row.extend([""] * n_cols)
                self._temp_csv_writer.writerow(row)
                self._temp_csv_file.flush()

    def _send_view_data(self):
        """发送可视化数据到 PlotjuggleDraw（旧版 send_view_data 静态方法的内联版）"""
        if self._sender is None:
            return

        current_state = self._state_machine.get_state()
        error_status = self._arm_ipc.get_error_status()

        device_key = f"dev{self._device_id}"
        data = {}

        # 电机位置
        if self._motor_pos is not None and hasattr(self._motor_pos, "tolist"):
            pos_list = self._motor_pos.tolist()
            for i, v in enumerate(pos_list):
                data[f"{device_key}/motor_position/joint{i}"] = float(v)

        # 目标位置
        if self._target_pos is not None:
            target = (self._target_pos.tolist()
                      if hasattr(self._target_pos, "tolist")
                      else self._target_pos)
            for i, v in enumerate(target):
                data[f"{device_key}/target_position/joint{i}"] = float(v)

        # 状态
        data[f"{device_key}/state"] = current_state.value
        data[f"{device_key}/error_code"] = error_status

        # session 信息
        if self._device is not None:
            data[f"{device_key}/ssid"] = self._device.get_my_session_id()
            data[f"{device_key}/holder"] = self._device.get_session_holder()

        self._sender.add_data(data)

    # ==================== 资源回收 ====================

    def _close(self):
        """finally 资源回收——CSV / Pipe / 报告 / API"""
        # 1. CSV 关闭
        if self._temp_csv_file:
            try:
                self._temp_csv_file.close()
            except Exception:
                pass

        # 2. Pipe 回收
        if self._arm_ipc is not None:
            try:
                if self._arm_ipc.cmd_recv_pipe.poll(timeout=0):
                    self._arm_ipc.cmd_recv_pipe.recv()
            except (EOFError, OSError):
                pass
            try:
                self._arm_ipc.cmd_recv_pipe.close()
            except Exception:
                pass

        # 3. 上报汇总数据
        if self._mp_queue is not None:
            try:
                pass
                report = {self._device_id: self._device_state.get_summary()}
                report[self._device_id].update({
                    "state": self._state_machine.get_state().value,
                    "loop_counter": self._loop_counter,
                })
                
                
                self._mp_queue.put(report)
            except Exception:
                pass

        # 4. 关闭 API
        if hasattr(self, '_hex_api') and self._hex_api is not None:
            try:
                self._hex_api.close()
            except Exception:
                pass
            
            
    # ==================== helper ====================

    @staticmethod
    def _parse_ws_url(url: str):
        """从 ws://host:port 简单分割解析 (host, port)

        用户指定"只做分割"：去掉 ws:// 前缀后按最后一个 ':' 切分。
        host 缺失时回退默认 192.168.1.100，port 缺失时回退默认 8439。
        注意：IPv6/interface 形式（如 ws://[::1%eth0]:8439）用简单分割不完整，
        非本轮目标，如需支持请改用 urlparse。
        """
        host = "192.168.1.100"
        port = 8439
        if not url:
            return host, port
        body = url[len("ws://"):] if url.startswith("ws://") else url
        if ":" in body:
            h, _, p = body.rpartition(":")
            if h:
                host = h
            if p.isdigit():
                port = int(p)
        else:
            if body:
                host = body
        return host, port
