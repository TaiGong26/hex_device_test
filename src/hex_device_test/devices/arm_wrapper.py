"""ArmWrapper — hex_driver_robot.robot 层后端实现

设计要点：
  - 按 params.robot_name 字符串分派，使用对应 HexRobot*Callback（Archer/Firefly）代替手动 HexDriver + Arm + work_loop 管理
  - Robot 层内部管理 HexDriver、Arm、Hands 设备创建及 work_loop
  - 自定义 callbacks 在 work_loop 线程内原子更新 _cache
  - motor_command / brake 委托给 robot 层 set_arm_pos_cmd / set_grip_pos_cmd
  - 命令持久化由 robot 层 __cur_cmd 机制保证（最后一个命令循环发送）

相比直接从 HexDriver + Arm 构建的好处：
  - 消除重复的 work_loop / proto 解析 / 命令队列管理代码
  - 自动处理多设备（Arm + Hands）协同
  - 减少与底层 API 的耦合，方便后续升级
"""

import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from hex_driver_robot import (
    HexRobotArcherY6Callback,
    HexRobotArcherY6Params,
    HexRobotFireflyY6Callback,
    HexRobotFireflyY6Params,
)

from .wrapper_base import WrapperBase, WrapperParams, GRIP_INFO_MAP


# === 软接受超时 ===
_RECV_TIMEOUT = 3.0          # 秒 — 接收超时判定
_API_EXIT_TIMEOUT = 15.0     # 秒 — 超过此时长无数据视为 API 已退出

# === 机器人分派表（key = robot_name 字符串）===
# 仅支持 6 轴带夹爪机械臂。Archer / Firefly 的 callback 与 Params 字段集完全一致，
# 因此只需按字符串切换 (Params 类, Callback 类)，其余 wrapper 代码共用接口。
ARM_ROBOT_DISPATCH = {
    "Archer_y6":  (HexRobotArcherY6Callback,  HexRobotArcherY6Params),
    "Firefly_y6": (HexRobotFireflyY6Callback, HexRobotFireflyY6Params),
}


class ArmWrapper(WrapperBase):
    """Arm 设备操作——hex_driver_robot.robot 后端实现

    生命周期：
        ArmWrapper(params)          → 按 params.robot_name 创建对应 HexRobot*Callback（阻塞等待首次数据）
        start()                     → 启动 robot 内部 work_loop 线程
        motor_command(...)/get_*()  → 正常控制/读取
        shutdown()                  → 停止 work_loop → 关闭连接

    机器人分派：仅支持 6 轴带夹爪机械臂（Archer_y6 / Firefly_y6）。
      直接按 params.robot_name 字符串查找 ARM_ROBOT_DISPATCH 创建对应 callback + param 类。
    """

    def __init__(self, logger=None, params: Optional[WrapperParams] = None):
        super().__init__(logger)
        if params is None:
            raise ValueError("params cannot be None, ArmWrapper initialization failed")
        self._params = params
        self._grip_dof = GRIP_INFO_MAP.get(self._params.grip_type, {}).get("motor_count", 0)
        self.init_vars()
        self.init_robot()

    # ==================================================================
    # 初始化
    # ==================================================================

    def init_vars(self):
        """设置成员变量（不涉及连接/IO）"""
        self._robot: Optional[Union[HexRobotArcherY6Callback,HexRobotFireflyY6Callback]] = None
        self._motor_count: int = 0

        # ── 状态缓存（callback 原子更新） ──
        self._cache_lock = threading.Lock()
        self._cache: Dict[str, Any] = {
            # Arm joint state
            "jnt_pos": None,
            "jnt_vel": None,
            "jnt_torque": None,
            # Arm temperature / error（从 robot.get_arm_motor_status 读取）
            "motor_temps": None,
            "driver_temps": None,
            "error_codes": None,
            # Grip joint state
            "grip_jnt_pos": None,
            "grip_jnt_vel": None,
            "grip_jnt_eff": None,
            # Grip temperature / error
            "grip_motor_temps": None,
            "grip_driver_temps": None,
            "grip_error_codes": None,
            # Session info（从 robot 内部 __arm 读取）
            "session_id": 0,
            "session_holder": 0,
        }
        self._cache_info: Dict[str, Any] = {
            "robot_mode": "",
            "overtoken_mode": "",    # robot 层不暴露 overtaken_reason，保持字段但留空
            "overtoken_reason": "",
        }

        # arm config mod
        self._config_mod = None

        self._last_recv_time: float = time.monotonic()

    # ── 日志（wrapper 使用 print，不依赖 logging）──
    # 继承自 WrapperBase: _log_info, _log_warn, _log_err, _log_debug

    # ==================================================================
    # 生命周期
    # ==================================================================

    def init_robot(self):
        """按 params.robot_name 字符串创建对应 HexRobot*Callback（阻塞等待首次数据），预分配缓存数组"""
        try:
            # ① 分派：robot_name → (CallbackCls, ParamsCls)
            robot_name = self._params.robot_name
            if robot_name not in ARM_ROBOT_DISPATCH:
                raise ValueError(
                    f"unknown robot_name '{robot_name}'; 支持: {list(ARM_ROBOT_DISPATCH)}"
                )
            callback_cls, params_cls = ARM_ROBOT_DISPATCH[robot_name]

            # ② 构建 robot 参数（Archer/Firefly 字段集一致，统一透传）
            robot_params = params_cls(
                host=self._params.host,
                port=self._params.port,
                ctrl_rate=self._params.ctrl_rate,
                state_buffer_size=self._params.state_buffer_size,
                sens_ts=self._params.sens_ts,
                grip_type=self._params.grip_type,
                enable_kcp=self._params.enable_kcp,
                init_timeout=30.0,
                log_level=self._params.log_level,
            )

            # ③ 创建 robot → __init__ 中 init_robot() 阻塞等待首次数据
            self._robot = callback_cls(
                params=robot_params,
                callbacks={
                    "arm_state": self._arm_state_cb,
                    "grip_state": self._grip_state_cb,
                },
            )

            # ④ 读取 DOF 信息
            dofs = self._robot.get_dofs()
            self._motor_count = dofs["arm"]

            # ⑤ 预分配 _cache 数组
            self._init_cache_arrays(self._motor_count, dofs.get("grip", 0))

            # ⑥ 启动 work_loop 线程（callback 的 __init__ 仅构建线程，不启动）
            self._robot.start()

            self._log_info("ArmWrapper initialized")

        except Exception as e:
            self._log_err(f"init_robot failed: {e}")
            raise

    def _init_cache_arrays(self, arm_dof: int, grip_dof: int):
        """预分配 _cache 中的 numpy 数组。"""
        with self._cache_lock:
            # Arm
            self._cache["jnt_pos"] = np.zeros(arm_dof)
            self._cache["jnt_vel"] = np.zeros(arm_dof)
            self._cache["jnt_torque"] = np.zeros(arm_dof)
            self._cache["motor_temps"] = np.full(arm_dof, np.nan)
            self._cache["driver_temps"] = np.full(arm_dof, np.nan)
            self._cache["error_codes"] = [[] for _ in range(arm_dof)]

            # Grip（如果存在）
            if grip_dof > 0:
                self._cache["grip_jnt_pos"] = np.zeros(grip_dof)
                self._cache["grip_jnt_vel"] = np.zeros(grip_dof)
                self._cache["grip_jnt_eff"] = np.zeros(grip_dof)
                self._cache["grip_motor_temps"] = np.full(grip_dof, np.nan)
                self._cache["grip_driver_temps"] = np.full(grip_dof, np.nan)
                self._cache["grip_error_codes"] = [[] for _ in range(grip_dof)]

        # 预分配完成后，在锁外同步读一次机械臂真实状态填入缓存。
        self._snapshot_current_state()

    def _snapshot_current_state(self):
        """预分配后同步读一次机械臂/夹爪真实状态填入缓存。

        消除 init 竞态：init_robot 返回前，work_loop 可能尚未派发第一次
        *_state_cb，此时 _cache 仍是占位全零。这里直接同步读底层缓存，
        保证 init 返回后 get_motor_positions() 立即返回真实位置（而非
        [0,0,0,0,0,0]），避免手柄被命令拉到错误的全零起步点。
        """
        if self._robot is None:
            return
        with self._cache_lock:
            arm = self._robot.get_arm_motor_status()
            if arm is not None:
                if arm.get("pos") is not None:
                    self._cache["jnt_pos"][:] = arm["pos"]
                if arm.get("vel") is not None:
                    self._cache["jnt_vel"][:] = arm["vel"]
                if arm.get("eff") is not None:
                    self._cache["jnt_torque"][:] = arm["eff"]
                if arm.get("motor_temp") is not None:
                    self._cache["motor_temps"][:] = arm["motor_temp"]
                if arm.get("driver_temp") is not None:
                    self._cache["driver_temps"][:] = arm["driver_temp"]
                if arm.get("error") is not None:
                    self._cache["error_codes"][:] = arm["error"]
            if self._cache.get("grip_jnt_pos") is not None:
                grip = self._robot.get_grip_motor_status()
                if grip is not None:
                    if grip.get("pos") is not None:
                        self._cache["grip_jnt_pos"][:] = grip["pos"]
                    if grip.get("vel") is not None:
                        self._cache["grip_jnt_vel"][:] = grip["vel"]
                    if grip.get("eff") is not None:
                        self._cache["grip_jnt_eff"][:] = grip["eff"]
                    if grip.get("motor_temp") is not None:
                        self._cache["grip_motor_temps"][:] = grip["motor_temp"]
                    if grip.get("driver_temp") is not None:
                        self._cache["grip_driver_temps"][:] = grip["driver_temp"]
                    if grip.get("error") is not None:
                        self._cache["grip_error_codes"] = [e.copy() for e in grip["error"]]

    def start(self) -> bool:
        """兼容性保留"""
        if self._robot is None or not self._robot.is_working():
            self._log_err("start() failed: ArmWrapper not in running state")
            return False
        return True

    def shutdown(self):
        """停止 work_loop 线程 → 关闭连接"""
        if self._robot is not None:
            self._robot.stop()
        self._log_info("ArmWrapper shutdown")

    # ==================================================================
    # Callbacks — 从 robot work_loop 线程调用，原子更新 _cache
    # ==================================================================

    def _arm_state_cb(self, state) -> None:
        """arm_state 回调：从 HexDcRoboArmStateStamped 缓存 arm 状态。

        Args:
            state: HexDcRoboArmStateStamped 实例（duck-typed）
        """
        motor_status = self._robot.get_arm_motor_status() if self._robot else None
        with self._cache_lock:
            self._last_recv_time = time.monotonic()
            # position / velocity / torque（来自 state 数据类）
            self._cache["jnt_pos"][:] = state.arm_state.jnt.position
            self._cache["jnt_vel"][:] = state.arm_state.jnt.velocity
            self._cache["jnt_torque"][:] = state.arm_state.jnt.effort
            # temperatures / errors（来自 motor_status）
            if motor_status is not None:
                if motor_status["motor_temp"] is not None:
                    self._cache["motor_temps"][:] = motor_status["motor_temp"]
                if motor_status["driver_temp"] is not None:
                    self._cache["driver_temps"][:] = motor_status["driver_temp"]
                if motor_status["error"] is not None:
                    self._cache["error_codes"][:] = motor_status["error"]
            # robot mode
            if self._robot is not None:
                self._cache_info["robot_mode"] = self._robot.get_arm_robot_mode() or ""

    def _grip_state_cb(self, state) -> None:
        """grip_state 回调：从 HexDcRoboGripStateStamped 缓存 grip 状态。

        Args:
            state: HexDcRoboGripStateStamped 实例（duck-typed）
        """
        grip_status = self._robot.get_grip_motor_status() if self._robot else None
        if grip_status is None:
            return
        with self._cache_lock:
            self._cache["grip_jnt_pos"][:] = state.grip_state.jnt.position
            self._cache["grip_jnt_vel"][:] = state.grip_state.jnt.velocity
            self._cache["grip_jnt_eff"][:] = state.grip_state.jnt.effort
            if grip_status["motor_temp"] is not None:
                self._cache["grip_motor_temps"][:] = grip_status["motor_temp"]
            if grip_status["driver_temp"] is not None:
                self._cache["grip_driver_temps"][:] = grip_status["driver_temp"]
            if grip_status["error"] is not None:
                self._cache["grip_error_codes"] = [e.copy() for e in grip_status["error"]]

    # ==================================================================
    # raw_device — 原始设备对象
    # ==================================================================

    @property
    def raw_device(self) -> Optional[Any]:
        """原始机器人对象（HexRobotArcherY6Callback）"""
        return self._robot

    @property
    def motor_count(self) -> int:
        """电机数量"""
        return self._motor_count

    def has_grip(self) -> bool:
        """是否有夹爪（按 grip_type 配置判断，不依赖回调时序）"""
        return self._grip_dof > 0

    # ==================================================================
    # API — 控制命令
    # ==================================================================

    def motor_command(self, command_type: str, values: Optional[List] = None):
        """Controller / State Machine 统一调用接口

        委托给 robot.set_arm_pos_cmd()，命令由 robot 层持久化自动重复。

        Args:
            command_type: "position" — 位置控制, values=[pos1, pos2, ...]
                          "brake"    — 模拟刹车，发送当前位置锁定
        """
        if self._robot is None:
            self._log_warn("motor_command called before robot is ready")
            return

        if command_type == "position":
            if values is None:
                return
            self._robot.set_arm_pos_cmd({
                "jnt_pos": [float(v) for v in values],
                "lim_vel": 10.0,
                "lim_acc": 10,
            })

        elif command_type == "brake":
            # "模拟刹车"：发送当前位置 → robot 层 __cur_cmd 持久化自动保持
            with self._cache_lock:
                current_pos = self._cache["jnt_pos"].copy()
            if current_pos is not None and len(current_pos) > 0:
                self._robot.set_arm_pos_cmd({
                    "jnt_pos": current_pos.tolist(),
                })
        else:
            self._log_err(f"Unknown command_type: {command_type}")

    # ==================================================================
    # API — Grip 控制
    # ==================================================================

    def grip_motor_command(self, command_type: str, values: Optional[List] = None):
        """夹爪控制接口

        Args:
            command_type: "position" — 位置控制, values=[pos]（夹爪通常单电机）
                          "brake"    — 模拟刹车，发送当前位置锁定
        """
        if self._robot is None:
            self._log_warn("grip_motor_command called before robot is ready")
            return

        if command_type == "position":
            if values is None or len(values) == 0:
                return
            pos = float(values[0])
            self._robot.set_grip_pos_cmd({"jnt_pos": [pos]})

        elif command_type == "brake":
            with self._cache_lock:
                current_pos = self._cache.get("grip_jnt_pos")
            if current_pos is not None and len(current_pos) > 0:
                self._robot.set_grip_pos_cmd({"jnt_pos": [float(current_pos[0])]})
        else:
            self._log_err(f"Unknown grip command_type: {command_type}")

    # ==================================================================
    # 状态读取 — Arm
    # ==================================================================

    def get_motor_positions(self) -> Optional[list]:
        """读取所有电机当前角度（弧度）"""
        with self._cache_lock:
            val = self._cache["jnt_pos"]
            return val.copy().tolist() if val is not None else None

    def get_motor_temperatures(self) -> Optional[list]:
        """(motor_temps, driver_temps)，每个元素 Optional[list]"""
        with self._cache_lock:
            val = self._cache["motor_temps"]
            return val.copy().tolist() if val is not None else None

    def get_motor_driver_temperatures(self) -> Optional[list]:
        """驱动温度列表"""
        with self._cache_lock:
            val = self._cache["driver_temps"]
            return val.copy().tolist() if val is not None else None

    def get_motor_error_codes(self) -> Optional[list]:
        """电机错误码列表（每个元素为列表，如 [['MeOverCurrent'], [], ...]）"""
        with self._cache_lock:
            val = self._cache["error_codes"]
            # error_codes 是 List[List[str]]，直接返回浅拷贝
            return [e.copy() for e in val] if val is not None else None

    # ==================================================================
    # 状态读取 — Grip
    # ==================================================================

    def get_grip_motor_positions(self) -> Optional[list]:
        """读取夹爪电机位置（弧度）"""
        with self._cache_lock:
            val = self._cache["grip_jnt_pos"]
            return val.copy().tolist() if val is not None else None

    def get_grip_motor_temperatures(self) -> Optional[list]:
        """夹爪电机温度列表"""
        with self._cache_lock:
            val = self._cache["grip_motor_temps"]
            return val.copy().tolist() if val is not None else None

    def get_grip_motor_driver_temperatures(self) -> Optional[list]:
        """夹爪驱动温度列表"""
        with self._cache_lock:
            val = self._cache["grip_driver_temps"]
            return val.copy().tolist() if val is not None else None

    def get_grip_motor_error_codes(self) -> Optional[list]:
        """夹爪电机错误码列表"""
        with self._cache_lock:
            val = self._cache["grip_error_codes"]
            return [e.copy() for e in val] if val is not None else None

    # ==================================================================
    # Session 信息
    # ==================================================================

    def get_session_info(self) -> Dict[str, Any]:
        """从 cache 读取 session 信息"""
        with self._cache_lock:
            return {
                "my_session_id": self._cache["session_id"],
                "session_holder": self._cache["session_holder"],
            }

    def get_my_session_id(self) -> int:
        with self._cache_lock:
            return self._cache["session_id"]

    def get_session_holder(self) -> int:
        with self._cache_lock:
            return self._cache["session_holder"]


    def get_robot_info(self) -> Dict:
        """arm 的 mode、overtaken 信息

        return: {
            robot_mode: str
            token_mode: str
            tokenreason: str
        }
        """
        return self._cache_info.copy()

    def get_arm_robot_mode(self):
        return self._robot.get_arm_robot_mode()