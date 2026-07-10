"""
ArmControllerMpV2 — 重构版控制器（Worker 模式）

设计意图：
将 ArmControllerProcess._task_loop（231 行）拆分为 6 个结构化方法，通过
run() 入口统一调度。IPC 保持 Pipe + mp.Value 不变（只修 poll 超时）。

run()
  ├── init_var()      —— 变量初始化
  ├── init_mod()      —— 模块创建（状态机 / API / 轨迹 / CSV / 可视化）
  ├── while loop:
  │   ├── error_check()   —— 设备发现 + 错误扫描
  │   ├── state_update()  —— Pipe 读取 + 状态机 dispatch
  │   └── run_tick()      —— 轨迹目标 + 温度 + CSV + View
  └── _close()        —— finally 资源回收

与旧版的关键差异：
  - 所有配置通过构造函数传入，无 setter
  - poll(timeout=0) 替代 poll(0.01)，消除 10ms 阻塞
  - daemon=True 移除
  - _consecutive_errors 计数，连续 N 次异常后退出（旧版无限重试）
  - PlotjuggleDraw 仅在 enable_view=True 时创建
"""

import time
import traceback
import csv
import os
from typing import Optional, List
import multiprocessing as mp

from hex_device import HexDeviceApi, Arm

from ..controllers.ErrorChecker import ArmErrorChecker
from ..tools.plotjuggle import PlotjuggleDraw
from .BaseController import BaseController
from .ArmControllerProcess import ArmStatusTable
from .TrajectoryController import TrajectoryPlanner, SegmentedTrajectoryPlanner
from ..tools.trajectory_loader import DEFAULT_SEGMENT_DURATION
from ..statuses.ArmProcessIPC import ArmCommChannel
from ..statuses.ArmStatus import ArmControllerStatus, ArmErrorStatus
from ..controllers.arm_state_machine_process import ArmControllerProcessStateMachine


class ArmControllerMpV2(BaseController):
    """重构版控制器——所有配置通过构造函数注入"""

    def __init__(self, ws_url: str, device_id: int = 0, enable_kcp: bool = False,
                 task_loop_hz: int = 500, arm_ipc: Optional[ArmCommChannel] = None,
                 arm_config: Optional[dict] = None,
                 waypoints: Optional[list] = None,
                 segment_duration: Optional[float] = None,
                 segment_ends: Optional[list] = None,
                 time_sleep: float = 0.0, interpolate: bool = True,
                 mp_queue: Optional[mp.Queue] = None,
                 temp_csv_dir: Optional[str] = None,
                 enable_view: bool = False, check_timeout: bool = False,
                 connect_timeout: float = 30.0,
                 max_consecutive_errors: int = 10):
        super().__init__(ws_url, 0, enable_kcp, task_loop_hz, device_id)

        # 模块/设备配置
        self._arm_ipc = arm_ipc
        self._arm_config = arm_config

        # 轨迹配置
        self._waypoints = waypoints
        self._segment_duration = segment_duration
        self._segment_ends = segment_ends
        self._time_sleep = time_sleep
        self._interpolate = interpolate

        # 运行配置
        self._mp_queue = mp_queue
        self._temp_csv_dir = temp_csv_dir
        self._enable_view = enable_view
        self._check_timeout = check_timeout
        self._connect_timeout = connect_timeout
        self._max_consecutive_errors = max_consecutive_errors

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
            print(f"[Device {self._device_id}, Exception]: {e}")
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
            print(f"[Controller {self._device_id}, RuntimeError]: {e}")
        except Exception as e:
            print(f"[Controller {self._device_id}, Exception]: {e}")

    # ==================== 子进程入口 ====================

    def run(self):
        """
        子进程入口（mp.Process(target=self.run) 启动）

        init_mod 在 try 内执行，失败则设 _loop_initialized=False，
        跳过主循环直接进入 finally _close()，确保资源回收。
        """
        self.init_var()

        try:
            self.init_mod()
            self._loop_initialized = True
        except Exception as e:
            print(f"[Dev {self._device_id}] init_mod failed: {e}")
            traceback.print_exc()
            self._loop_initialized = False

        try:
            if not self._loop_initialized:
                return

            while not self._loop_running.is_set():
                try:
                    self.error_check()

                    # ── 设备未发现：累计超时 + 睡眠 + 跳过 ──
                    if self._device is None:
                        self._connect_timeout_elapsed += self._task_interval
                        # ⚠️ 修复：超时后检查 _exit_requested，避免死循环
                        self._check_connect_timeout()
                        if self._exit_requested:
                            break
                        time.sleep(self._task_interval)
                        continue

                    self.state_update()

                    if self._exit_requested:
                        break

                    self.run_tick()
                    time.sleep(self._task_interval)

                except KeyboardInterrupt:
                    # 子进程中忽略 Ctrl+C（由父进程信号处理控制生命周期）
                    pass

                except Exception as e:
                    self._consecutive_errors += 1
                    print(f"[Dev {self._device_id}] loop exception: {e}")
                    traceback.print_exc()
                    # 保留旧版 IPC 错误信号，让 Coordinator 能感知子进程异常
                    if self._arm_ipc is not None:
                        self._arm_ipc.set_error_status(ArmErrorStatus.ProcessError.value)
                    if self._consecutive_errors >= self._max_consecutive_errors:
                        print(f"[Dev {self._device_id}] too many consecutive errors, exit")
                        break
        finally:
            self._close()

    # ==================== 方法拆分 ====================

    def init_var(self):
        """变量初始化（只赋初值，无需回收）"""
        self._task_interval = 1.0 / self._task_loop_hz
        self._device: Optional[Arm] = None
        self._trajectory = None
        self._target_pos = None
        self._motor_pos = None
        self._last_pos = None
        self._connect_timeout_elapsed = 0.0
        self._loop_counter = 0
        self._prev_segment_index = None
        self._last_temp_log_time = 0.0
        self._consecutive_errors = 0
        self._exit_requested = False
        self._loop_initialized = False

    def init_mod(self):
        """模块创建（状态机 / API / 轨迹 / CSV / 可视化）"""
        # 状态机
        self._state_machine = ArmControllerProcessStateMachine(
            self._device_id, self._arm_ipc
        )
        # 设备状态表（温度 / 错误记录）
        self._device_state = ArmStatusTable()

        # 可视化（仅开启时创建 PlotjuggleDraw UDP sender）
        self._sender = None
        if self._enable_view:
            self._sender = PlotjuggleDraw()
            self._sender.start()

        # HexDeviceApi
        self._hex_api = HexDeviceApi(
            ws_url=self._ws_url,
            local_port=0,
            enable_kcp=self._enable_kcp,
        )

        # 轨迹规划器
        if self._waypoints:
            duration = (self._segment_duration
                        if self._segment_duration is not None
                        else DEFAULT_SEGMENT_DURATION)
            if self._segment_ends:
                self._trajectory = SegmentedTrajectoryPlanner(
                    waypoints=self._waypoints,
                    segment_ends=self._segment_ends,
                    segment_duration=duration,
                    hold_duration=self._time_sleep if self._time_sleep is not None else 0.0,
                    interpolate=self._interpolate,
                )
                print(
                    f"dev{self._device_id}: segmented trajectory, "
                    f"{len(self._segment_ends)} segment(s), "
                    f"segment_duration={duration}s, time_sleep={self._time_sleep}s, "
                    f"interpolate={self._interpolate}, "
                    f"segment_ends={self._segment_ends}"
                )
            else:
                self._trajectory = TrajectoryPlanner(
                    waypoints=self._waypoints,
                    segment_duration=duration,
                    interpolate=self._interpolate,
                )
                print(
                    f"dev{self._device_id}: trajectory segment_duration={duration}s, "
                    f"interpolate={self._interpolate}"
                )

        # CSV 日志文件（header 推迟到 error_check 发现设备后写入）
        self._temp_csv_file = None
        self._temp_csv_writer = None
        if self._temp_csv_dir:
            os.makedirs(self._temp_csv_dir, exist_ok=True)
            start_ts = time.strftime("%Y-%m-%d_%H-%M-%S")
            csv_path = os.path.join(
                self._temp_csv_dir,
                f"temp_dev{self._device_id}_{start_ts}.csv"
            )
            self._temp_csv_file = open(csv_path, 'w', newline='')
            self._temp_csv_writer = csv.writer(self._temp_csv_file)
            print(f"dev{self._device_id}: temperature CSV -> {csv_path}")

    def error_check(self):
        """
        设备发现 + 错误扫描

        分四步：
        1. 设备发现（仅一次）—— 找到第一个 Arm 实例
        2. API 退出检查
        3. 连接丢失检查
        4. ArmErrorChecker 全面错误扫描 → Brake
        """
        # ── 1. 设备发现（仅一次） ──
        if self._device is None:
            for dev in self._hex_api.device_list:
                if isinstance(dev, Arm):
                    self._device = dev
                    if not self._device.reload_arm_config_from_dict(self._arm_config):
                        self._state_machine.transition(
                            ArmControllerStatus.Exit,
                            f"errors: device{self._device_id} not arm config"
                        )
                    print(f"dev{self._device_id}: robot_type{self._device.robot_type}")

                    # CSV header：设备发现后用实际 motor_count 动态生成
                    if self._temp_csv_writer is not None:
                        n_motors = self._device.motor_count
                        header = ["timestamp"] + [f"motor_{i}" for i in range(n_motors)]
                        self._temp_csv_writer.writerow(header)
                        self._temp_csv_file.flush()
                    break

            if self._device is None:
                return  # 调用方感知 None 后处理超时 + continue

        # ── 2. API 退出检查 ──
        if self._hex_api.is_api_exit():
            self._exit_requested = True
            return

        # ── 3. 连接丢失检查 ──
        conn_lost = self._hex_api.is_websocket_recv_timeout()

        # ── 4. 错误扫描 → Brake ──
        has_error, errors = ArmErrorChecker.check_device(
            self._check_timeout, self._device, conn_lost
        )
        if has_error:
            error_codes = [err_tuple[0] for err_tuple in errors]
            min_error_code = min(error_codes, key=lambda x: x.value)
            self._arm_ipc.set_error_status(min_error_code.value)

            error_details = []
            for code, reasons in errors:
                info = ArmErrorChecker.format_error(code, reasons)
                error_details.append(f"{code.name}: {info}")
                self._device_state.set_error(code, info)

            final_error_msg = " | ".join(error_details)
            self._state_machine.transition(
                ArmControllerStatus.Brake,
                f"errors: {final_error_msg}"
            )

    def _check_connect_timeout(self):
        """连接超时检查——累计超过阈值则标记退出"""
        if self._connect_timeout_elapsed >= self._connect_timeout:
            print(f"[Dev {self._device_id}] connect timeout "
                  f"({self._connect_timeout}s), exit")
            self._exit_requested = True

    def state_update(self):
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
            self._device._last_command_time = None

        elif current_state == ArmControllerStatus.Running:
            # ⚠️ _target_pos 由上一个 tick 的 run_tick() 计算
            # 这引入 ~1 frame（~2ms @ 500Hz）延迟，机械上无感知
            self._state_machine.handle_running(self._device, self._target_pos)
            if self._trajectory:
                self._update_loop_count()

        elif current_state == ArmControllerStatus.Stopped:
            self._state_machine.handle_stopped(self._device, self._last_pos)

        elif current_state == ArmControllerStatus.Brake:
            self._state_machine.handle_brake(self._device)

        elif current_state == ArmControllerStatus.Exit:
            self._state_machine.handle_exit()
            self._exit_requested = True

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

    def run_tick(self):
        """
        轨迹目标计算 + 温度采集 + CSV 日志 + 可视化

        _target_pos 在本 tick 计算，供下一个 tick 的 state_update() 使用。
        这是与旧代码的时序差异——旧代码 target_pos 在 state machine 之前计算。
        """
        # ── 1. 轨迹目标 + 电机位置 ──
        self._target_pos = (self._trajectory.get_current_target()
                            if self._trajectory else None)
        self._motor_pos = self._device.get_motor_positions()
        self._last_pos = (self._trajectory.get_last_position()
                          if self._trajectory else None)

        # ── 2. 可视化数据 ──
        if self._enable_view and self._sender is not None:
            self._send_view_data()

        # ── 3. 电机温度 + 驱动温度 ──
        motor_temps = self._device.get_motor_temperatures()
        driver_temps = self._device.get_motor_driver_temperatures()
        self._device_state.update(motor_temps, driver_temps)

        # ── 4. CSV 日志（~1Hz） ──
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

        # 2. Pipe 排空 + 关闭
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
                report = {self._device_id: self._device_state.get_summary()}
                report[self._device_id].update({
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
