"""ArmWrapper — hex_driver_robot 后端实现（A2 daemon thread 方案）

设计要点：
  - Daemon thread 全权版：drain_up → try_update → tick_control → flush_down_once
  - motor_command("position"/"brake", values) 统一接口，保持 Controller 调用方式
  - 无 discover/device_list 层，init_robot + Arm 创建在 start() 内完成
  - 临时数据 shim（TODO: remove when hex_driver_robot officially supports）
  - 所有不确定决策用 === DECIDE(?) === 标记
"""

import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from hex_util_runtime import HexRate
import numpy as np

from hex_driver_robot.tcp_base.ipc import UpQueueItem

from .wrapper_base import WrapperBase, WrapperParams, ARM_INFO_MAP, ARM_ROBOT_TYPE_MAP

from hex_driver_robot.device.arm import Arm
from hex_driver_robot.tcp_base import HexDriver


# === DECIDE(kp_kd): kp/kd 默认值需要根据实际机械臂确定 ===
# 当前值是从 hex_driver_robot 参考代码中取的保守默认值
_KP_DEFAULT = 20.0   # 位置刚度
_KD_DEFAULT = 0.5    # 阻尼系数
# === END DECIDE(kp_kd) ===


# === 软接受超时 ===
_RECV_TIMEOUT = 3.0  # 秒

# === DECIDE(shutdown_timeout): daemon 线程 join 超时 ===
_SHUTDOWN_JOIN_TIMEOUT = 3.0  # 秒
# === END DECIDE(shutdown_timeout) ===


@dataclass
class ParkingStopDetail:
    """停车详情——proto v2.0 无此上行字段，始终返回默认值

    hex_driver_robot 的 ArmStatus proto 定义（public_api_types.proto）
    没有 parking_stop_detail 字段。旧 hex_device v1.4.x 的 proto 中有，
    但新 v2.0 协议用 overtaken_reason（APIUp 顶层字段）+ robot_mode
    替代了该功能。

    始终返回 category=0（无停车），ErrorChecker 据此不触发停车错误。
    如果未来 proto 升级加回该字段，在 __update_state_cache 中
    解析 item.raw 并填充即可。
    """
    category: int = 0
    reason: str = ""
    is_remotely_clearable: bool = False


class ArmWrapper(WrapperBase):
    """Arm 设备操作——hex_driver_robot 后端实现

    生命周期：
        ArmWrapper(params)          → 创建 HexDriver（不启动连接）
        start()                     → HexDriver.start() + init_robot + daemon thread
        motor_command(...)/get_*()  → 正常控制/读取
        shutdown()                  → 停线程 → HexDriver.close()
    """

    def __init__(self, logger=None, params: Optional[WrapperParams] = None):
        super().__init__(logger)
        if params is None:
            raise ValueError("params cannot be None, ArmWrapper initialization failed")
        self._params = params
        self.init_vars()
        self.init_robot()

    # ==================================================================
    # 初始化（robot.py 风格）
    # ==================================================================

    def init_vars(self):
        """设置成员变量（不涉及连接/IO）"""
        self._hex_driver = None

        self._robot: Optional[Arm] = None
        self._motor_count: int = 0
        self._info = {"robot_type": 0}

        # Daemon thread 控制
        self._stop_event = threading.Event()
        self._work_thread = threading.Thread(target=self._work_loop, daemon=True)

        # ── 状态缓存（robot.py 风格：全 np.ndarray + 预分配 + 原地 slice 写入）──
        self._cache_lock = threading.Lock()
        self._cache: Dict[str, Any] = {
            # Joint state（从 get_simple_motor_status 更新）
            "jnt_pos": None,       # Optional[np.ndarray]
            "jnt_vel": None,       # Optional[np.ndarray]
            "jnt_torque": None,    # Optional[np.ndarray]
            # Temperature / error（从 proto 解析）
            "motor_temps": None,   # Optional[np.ndarray]
            "driver_temps": None,  # Optional[np.ndarray]
            "error_codes": None,   # Optional[np.ndarray]
            # Session info（从 item.common 更新，纯 int 无需预分配）
            "session_id": 0,
            "session_holder": 0,
        }
        self._last_recv_time: float = time.monotonic()

        # ── 线程错误追踪 ──
        # === DECIDE(thread_error): 线程内异常的传递方式 ===
        # 当前方案：daemon 线程静默记日志 + 设 _thread_error 标记
        # 备选方案：设连续错误计数，超限自动停止线程
        # self._thread_error: Optional[str] = None
        # === END DECIDE(thread_error) ===

    # ── 日志（wrapper 使用 print，不依赖 logging）──
    # 继承自 WrapperBase: _log_info, _log_warn, _log_err, _log_debug

    # ==================================================================
    # 生命周期
    # ==================================================================

    def init_robot(self):
        """
            -> 
        """
        try:
            self._hex_driver = HexDriver(
                f"ws://{self._params.host}:{self._params.port}",
                control_hz=int(self._params.ctrl_rate),
                enable_kcp=self._params.enable_kcp,
                log_level=self._params.log_level,
            )
            
            # ① 启动 NetworkWorker 子进程
            self._hex_driver.start()

            # ② 从上游获取 robot type
            self._sync_robot_type()

            # ③ 创建 Arm
            info = ARM_INFO_MAP.get(self._params.robot_name)
            if info is None:
                raise ValueError(f"Unknown robot_name: {self._params.robot_name}")

            self._motor_count = info["motor_count"]

            # 预分配 cache 数组（robot.py 风格：固定大小 + 原地 slice 写入）
            self._cache["jnt_pos"] = np.zeros(self._motor_count)
            self._cache["jnt_vel"] = np.zeros(self._motor_count)
            self._cache["jnt_torque"] = np.zeros(self._motor_count)
            self._cache["motor_temps"] = np.zeros(self._motor_count)
            self._cache["driver_temps"] = np.zeros(self._motor_count)
            self._cache["error_codes"] = np.zeros(self._motor_count, dtype=int)

            self._robot = Arm(
                self._hex_driver,
                robot_type=info["robot_type"],
                motor_count=self._motor_count,
                control_hz=int(self._params.ctrl_rate),
            )

            # ④ readiness probe: 阻塞等待第一个上行数据包
            probe_timeout = 30.0
            probe_start = time.time()
            while not self._stop_event.is_set():
                if not self._hex_driver.is_connected():
                    raise ConnectionError("HexDriver connection failed during init_robot")

                items = self._hex_driver.drain_up()
                if items:
                    for item in items:
                        self._robot.try_update(item)
                    if self._robot.cache_positions is not None:
                        self._log_info("init_robot: first motor data received")
                        break

                if time.time() - probe_start > probe_timeout:
                    raise TimeoutError(
                        f"init_robot timeout ({probe_timeout}s): no motor data"
                    )
                time.sleep(0.01)

            # ⑤ 启动 daemon thread
            self._work_thread.start()
            self._log_info("ArmWrapper initialized")

        except Exception as e:
            self._log_err(f"init_robot failed: {e}")
            raise

    def start(self) -> bool:
        """兼容性保留"""
        if self._robot is None or not self._work_thread.is_alive():
            self._log_err("start() failed: ArmWrapper not in running state")
            return False
        return True

    def shutdown(self):
        """关闭 HexDriver"""
        self._stop_event.set()

        if self._work_thread.is_alive():
            self._work_thread.join(timeout=_SHUTDOWN_JOIN_TIMEOUT)
            if self._work_thread.is_alive():
                self._log_warn("daemon thread did not exit within timeout")

        self._hex_driver.close()
        self._log_info("ArmWrapper shutdown")

    def _sync_robot_type(self):
        """在初始化ARM前，从上游获取robot_type
            - 因为Arm()内有 item.common.robot_type != self._robot_type:
        """
        
        init_deadline = _RECV_TIMEOUT + time.monotonic()
        
        while self._info["robot_type"] == 0:
            if self._hex_driver is not None:
                items = self._hex_driver.drain_up()
                for item in items:
                    com = item.common
                    # 同步捕获 session 信息
                    with self._cache_lock:
                        self._cache["session_id"] = com.session_id
                        self._cache["session_holder"] = com.session_holder
                    if com.robot_type in ARM_ROBOT_TYPE_MAP.values():
                        self._info["robot_type"] = com.robot_type
                        return

            if time.monotonic() > init_deadline:
                raise TimeoutError("[Arm wrapper]: recv timeout, from '_sync_robot_type' function")
            
            time.sleep(0.01)

    # ==================================================================
    # Daemon thread — 照抄 robot.py work_loop
    # ==================================================================

    def _work_loop(self):
        """Daemon 线程入口

        每 cycle 按序执行：
            ① drain_up()     — 取所有上行数据
            ② try_update()   — 解析 protobuf → 填充 MotorCacheMixin + 临时数据
            ③ tick_control() — 首次发 placeholder（_last_command_time is None）
            ④ flush_down_once() — 发下行（取 _down_queue 最新命令）
        """
        
        rate = HexRate(self._params.ctrl_rate)
        
        while not self._stop_event.is_set():
            rate.sleep()
            
            try:
                # ── ① 收上行 ──
                items = self._hex_driver.drain_up()

                # ── ② 更新缓存 ──
                if items:
                    self._last_recv_time = time.monotonic()
                    for item in items:
                        self.__update_state_cache(item)

                elif not self._hex_driver.is_connected():
                    # 网络已断开，停止循环
                    self._log_warn("HexDriver disconnected, stopping daemon thread")
                    break

                # ── ④ tick_control — 首次 placeholder ──
                self._robot.tick_control()

                # ── ⑤ flush_down_once — 发下行 ──
                self._robot.flush_down_once()

            except Exception as e:
                # === DECIDE(thread_error): 线程异常处理 ===
                # 当前：记日志 + 设标记，不崩溃线程
                # 备选：连续错误计数 N 次后 break
                self._log_err(f"work_loop error: {e}")
                # self._thread_error = str(e)
                # === END DECIDE(thread_error) ===


        #  self._log_info("daemon thread exited")
    def __update_state_cache(self, item: UpQueueItem):
        """合并更新 Arm 内部缓存 + wrapper _cache，持锁写入（robot.py 风格）

        try_update → 更新 Arm 的 MotorCacheMixin（pos/vel/eff）
        get_simple_motor_status → 读取 Arm 缓存 → 原地 slice 写入 _cache
        解析 raw proto → np.array(temp/error) → 原地 slice 写入 _cache
        """
        if not self._robot.try_update(item):
            return

        # ① 从 Arm 内部缓存读取 pos/vel/torque
        motor_status = self._robot.get_simple_motor_status()

        # ② 解析 proto 提取 temp/error → np.ndarray
        from hex_driver_robot.tcp_base.generated import public_api_up_pb2

        motor_temps: Optional[np.ndarray] = None
        driver_temps: Optional[np.ndarray] = None
        error_codes: Optional[np.ndarray] = None
        try:
            api_up = public_api_up_pb2.APIUp()
            api_up.ParseFromString(item.raw)
            if api_up.HasField('arm_status'):
                ms_list = api_up.arm_status.motor_status
                motor_temps = np.array([ms.motor_temperature for ms in ms_list])
                driver_temps = np.array([ms.driver_temperature for ms in ms_list])
                error_codes = np.array([
                    ms.error_code if ms.HasField('error_code') else 0
                    for ms in ms_list
                ], dtype=int)
        except Exception:
            pass

        # ③ 持锁 → 原地 slice 写入（零分配，对齐 robot.py 风格）
        with self._cache_lock:
            # Session info（每个上行包都带）
            self._cache["session_id"] = item.common.session_id
            self._cache["session_holder"] = item.common.session_holder
            if motor_status is not None:
                self._cache["jnt_pos"][:] = motor_status["pos"]
                self._cache["jnt_vel"][:] = motor_status["vel"]
                self._cache["jnt_torque"][:] = motor_status["eff"]
            if motor_temps is not None:
                self._cache["motor_temps"][:] = motor_temps
                self._cache["driver_temps"][:] = driver_temps
                self._cache["error_codes"][:] = error_codes

    # ==================================================================
    # 控制命令 — 统一 motor_command(type, values) 接口
    # ==================================================================

    def motor_command(self, command_type: str, values:Optional[List]=None):
        """Controller/State Machine 统一调用接口

        Args:
            command_type: "position" — 位置控制, values=[pos1, pos2, ...]
                          "brake"    — 刹车, values 忽略
        """
        if self._robot is None:
            self._log_warn("motor_command called before robot is ready")
            return


        # ##TODO: 按照robot风格重写这个motorcommand，需要查询相关proto字段对于 brake的定义。
        if command_type == "position":
            pass
            # if values is None:
            #     return
            # commands = self._robot.construct_mit_command(
            #     pos=values,
            #     speed=np.zeros(self._motor_count),
            #     torque=np.zeros(self._motor_count),
            #     kp=np.full(self._motor_count, _KP_DEFAULT),
            #     kd=np.full(self._motor_count, _KD_DEFAULT),
            # )
            # self._robot.motor_command("mit", commands)

        elif command_type == "brake":
            # self._robot.motor_command("brake", [])
            pass

        else:
            self._log_err(f"Unknown command_type: {command_type}")


    def reset_last_command_time(self) -> None:
        """令 tick_control() 重新发 placeholder

        调用时机：Controller 进入 Ready 等不发送位置命令的稳定状态。
        效果：daemon thread 的 tick_control() 检测到 _last_command_time is None，
              每 cycle 发 placeholder 保持连接，直到下次 motor_command()。
        """
        if self._robot is not None:
            self._robot._last_command_time = None


    # ==================================================================
    # 状态读取 — 接口保持
    # ==================================================================

    def get_motor_positions(self) -> Optional[list]:
        with self._cache_lock:
            val = self._cache["jnt_pos"]
            return val.tolist() if val is not None else None

    def get_motor_temperatures(self) -> tuple:
        """(motor_temps, driver_temps)，每个元素 Optional[list]"""
        with self._cache_lock:
            mt = self._cache["motor_temps"]
            if mt is None:
                return (None, None)
            return mt.tolist() if mt is not None else None
        
    def get_motor_driver_temperatures(self) -> Optional[list]:
        with self._cache_lock:
            val = self._cache["driver_temps"]
            return val.tolist() if val is not None else None

    def get_motor_error_codes(self) -> Optional[list]:
        with self._cache_lock:
            val = self._cache["error_codes"]
            return val.tolist() if val is not None else None


    def get_session_info(self) -> Dict[str, Any]:
        """从 cache 读取 session 信息"""
        with self._cache_lock:
            return {
                "my_session_id": self._cache["session_id"],
                "session_holder": self._cache["session_holder"],
            }

    # ── Controller 用到的 session 方法 ──

    def get_my_session_id(self) -> int:
        with self._cache_lock:
            return self._cache["session_id"]

    def get_session_holder(self) -> int:
        with self._cache_lock:
            return self._cache["session_holder"]

    # ==================================================================
    # 错误检查 — 保持 Controller 现有调用签名
    # ==================================================================

    def is_api_exit(self) -> bool:
        """API 是否已退出

        映射到 HexDriver 连接状态 + daemon thread 状态。
        """
        if not self._hex_driver.is_connected():
            return True
        if not self._work_thread.is_alive():
            return True
        # if self._thread_error is not None:
        #     return True
        return False

    def is_websocket_recv_timeout(self) -> bool:
        """接收超时

            - 超过 _RECV_TIMEOUT 秒未收到任何上行数据，判为超时
        """
        return (time.monotonic() - self._last_recv_time) > _RECV_TIMEOUT


    def get_parking_stop_detail(self) -> ParkingStopDetail:
        """停车详情——hex_driver_robot 不含此字段，始终返回默认值"""
        # hex_driver_robot v0.1 的 arm_status proto 没有 parking_stop_detail 字段
        # 返回默认 ParkingStopDetail() → category=0, ErrorChecker 判定无错误
        return ParkingStopDetail()
