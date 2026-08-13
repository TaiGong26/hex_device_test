"""
ArmCoordinatorV2 — 重构版协调器

关键变化：
  - 无 setter 阶段：Controller 构造时注入全部配置
  - write_csv 改用 temp_csv_dir 参数，不依赖硬编码路径
  - 串行 shutdown（比旧版并发 shutdown 更可靠）
  - IPC 架构不变：Pipe + mp.Value
"""

import logging
import traceback
from typing import Optional, List, Dict
import threading
import time
import multiprocessing as mp
import os
from collections import deque

from .BaseCoordinator import BaseCoordinator
from ..tools.CsvLogger import write_csv
from ..controllers.ArmControllerMpV2 import ArmControllerMpV2 as Controller
from ..controllers.arm_state_machine_process import ArmCoordinatorProcessStateMachine
from ..statuses.ArmStatus import ArmCmdStatus, ArmCoordinatorStatus, ArmControllerStatus, ArmErrorStatus
from ..statuses.ArmProcessIPC import ArmCommChannelManager


class ArmCoordinatorV2(BaseCoordinator):

    def __init__(self, device_ws_url_list: Optional[List[str]] = None,
                 enable_kcp: bool = False,
                #  arm_config: Optional[dict] = None,
                 robot_type: str = "Archer_y6",
                 waypoints: Optional[List[list]] = None,
                 segment_duration: Optional[float] = None,
                 interpolate: str = 'linear',
                 timestamps: Optional[List[float]] = None,
                 enable_view: bool = False,
                 temp_csv_dir: Optional[str] = None):
        super().__init__()

        self._logger = logging.getLogger("Coordinator")

        # 保存配置
        self._enable_kcp = enable_kcp
        self._robot_type = robot_type
        self._waypoints = waypoints
        self._segment_duration = segment_duration
        self._interpolate = interpolate
        self._timestamps = timestamps
        self._enable_view = enable_view
        self._temp_csv_dir = temp_csv_dir

        # 进程通信管理
        self._arm_ipc = ArmCommChannelManager()
        self._mp_queue = mp.Queue()

        # 状态机
        self._state_machine = ArmCoordinatorProcessStateMachine(self)

        # 运行时数据
        self._controllers_list: List[Controller] = []
        self._device_states: Dict[int, ArmControllerStatus] = {}
        self._error_states: Dict[int, ArmErrorStatus] = {}
        self._error_flag: List[bool] = []

        # 监控线程
        self._task: Optional[threading.Thread] = None

        # 启动
        self._start(device_ws_url_list, enable_kcp, robot_type)

    def _start(self, device_ws_url_list, enable_kcp, robot_type):
        """
        （内部）启动所有控制器

        与旧版的关键差异：
        - 控制器在构造时注入全部配置，无需 11 个 setter
        - 创建和启动在同一循环完成
        """
        if device_ws_url_list is None:
            self._logger.warning("device ip list is None")
            return False

        # 创建控制器（一步到位，所有配置通过构造函数传入）
        for idx, ip in enumerate(device_ws_url_list):
            device_ipc = self._arm_ipc.create_arm_ipc(idx)

            controller = Controller(
                ws_url=ip,
                device_id=idx,
                enable_kcp=enable_kcp,
                task_loop_hz=500,
                arm_ipc=device_ipc,
                robot_type=robot_type,
                waypoints=self._waypoints,
                timestamps=self._timestamps,
                segment_duration=self._segment_duration,
                interpolate=self._interpolate,
                enable_view=self._enable_view,
                mp_queue=self._mp_queue,
                temp_csv_dir=self._temp_csv_dir,
            )
            self._controllers_list.append(controller)

        self._logger.info("controllers %d", len(self._controllers_list))
        self._error_flag = [False] * len(self._controllers_list)

        # 启动所有控制器
        for controller in self._controllers_list:
            controller.start()

        # 启动监控线程（20Hz）
        self._task = threading.Thread(target=self._task_loop)
        self._task.start()

    # ==================== shutdown ====================

    def shutdown(self):
        """
        停止所有控制器（5 阶段）

        相比旧版的变化：
          - 串行 shutdown（旧版为每个 controller 创建独立线程并发 shutdown）
          - Phase 4：drain mp_queue 并 write_csv 到 temp_csv_dir
        """
        # Phase 1: 状态机 → Stopped
        self._state_machine.transition_to(ArmCoordinatorStatus.Stopped, "shutdown")

        # Phase 2: 等待子进程 Exit（最多 12s）
        stopped_time = 0
        while self._state_machine._state != ArmCoordinatorStatus.Exit:
            time.sleep(0.1)
            stopped_time += 0.1
            if stopped_time >= 12:
                break

        # Phase 3: 并发 shutdown
        shutdown_threads = []
        with self.controller_lock:
            for controller in self._controllers_list:
                t = threading.Thread(target=controller.shutdown)
                t.start()
                shutdown_threads.append(t)
        for t in shutdown_threads:
            t.join(timeout=5.0)
            
        # Phase 4: 清理
        if self._task:
            self._task.join(timeout=0.1)
            self._task = None

        # Phase 5: controller 写入 CSV 文件（用 temp_csv_dir，不依赖硬编码路径）
        if self._temp_csv_dir:
            os.makedirs(self._temp_csv_dir, exist_ok=True)
            t = time.strftime("%Y-%m-%d_%H-%M-%S")
            CSV_PATH = os.path.join(self._temp_csv_dir, f"arm_test_{t}.csv")
            write_csv(self._mp_queue, CSV_PATH)
            self._logger.info("report written to %s", CSV_PATH)

        self._ipc_clean()
        self._mp_queue.close()
        self._stop_event.set()
        self._logger.info("-------------------------------- Process shutdown ----------------------------------")

    def _ipc_clean(self):
        """清理 IPC 通道"""
        try:
            self._arm_ipc.cleanup_all()
        except Exception as e:
            self._logger.error("IPC clean err: %s", e)

    # ==================== 命令接口 ====================

    def publish_command(self, cmd: int):
        """向所有设备广播命令"""
        try:
            ipc_dict = self._arm_ipc.get_ipc_dict()
            for device_id, ipc in ipc_dict.items():
                if not ipc.cmd_send_pipe.closed:
                    if isinstance(cmd, int):
                        ipc.cmd_send_pipe.send(cmd)
                    elif isinstance(cmd, ArmCmdStatus):
                        ipc.cmd_send_pipe.send(cmd.value)
            self._logger.info("send command: %s", ArmCmdStatus(cmd).name)
        except Exception as e:
            self._logger.error("命令发送异常: %s", e)

    def publish_dev_command(self, dev_id: int, cmd: int):
        """向指定设备发送命令"""
        try:
            ipc = self._arm_ipc.get_device_ipc(dev_id)
            if ipc and not ipc.cmd_send_pipe.closed:
                if isinstance(cmd, int):
                    ipc.cmd_send_pipe.send(cmd)
                elif isinstance(cmd, ArmCmdStatus):
                    ipc.cmd_send_pipe.send(cmd.value)
            self._logger.info("send command to dev%d: %s", dev_id, ArmCmdStatus(cmd).name)
        except Exception as e:
            self._logger.error("命令发送异常: %s", e)

    # ==================== 状态查询 ====================

    def get_error_flag(self) -> List[bool]:
        """返回错误标志副本"""
        return self._error_flag.copy()

    def get_all_controller_status(self) -> List[ArmControllerStatus]:
        """获取所有子进程状态（从 IPC 共享内存读取）"""
        return [
            ArmControllerStatus(ipc.get_controller_status())
            for ipc in self._arm_ipc.get_ipc_dict().values()
        ]

    def check_any_device_error(self) -> tuple:
        """检查是否有设备报错"""
        for dev_id, ipc in self._arm_ipc.get_ipc_dict().items():
            status = ArmErrorStatus(ipc.get_error_status())
            if status.value > ArmErrorStatus.Warning.value:
                return True, f"dev{dev_id} {status.name}"
        return False, ""

    def has_pending_command(self, cmd: ArmCmdStatus) -> bool:
        """检查是否所有设备都收到了指定命令"""
        return any(
            ipc.get_cmd_status() == cmd.value
            for ipc in self._arm_ipc.get_ipc_dict().values()
        )

    def get_device_error_reason(self, dev_id: int) -> str:
        """获取指定设备的错误原因"""
        arm_ipc = self._arm_ipc.get_device_ipc(dev_id)
        if arm_ipc is None:
            return "[From Coordinator]: 没有设备IPC通道, 无法查询"
        state = ArmErrorStatus(arm_ipc.get_error_status())
        return state.name

    # ==================== 监控循环 ====================

    def _task_loop(self):
        """协调器主循环 - 20Hz"""
        try:
            while not self._stop_event.is_set():
                # 1. 扫描设备状态共享内存
                self._scan_device_states()

                # 2. 状态机步进
                self._state_machine.step(self.get_error_flag())

                time.sleep(0.01)  # 20Hz
        except Exception as e:
            self._logger.error("主循环异常: %s", e)
            traceback.print_exc()

    def _scan_device_states(self):
        """扫描所有子进程的共享内存状态"""
        ipc_dict = self._arm_ipc.get_ipc_dict()

        for device_id, ipc in ipc_dict.items():
            error_status = ArmErrorStatus(ipc.get_error_status())
            controller_status = ArmControllerStatus(ipc.get_controller_status())

            # 更新本地缓存
            self._device_states[device_id] = controller_status
            self._error_states[device_id] = error_status

            # 检查错误
            if error_status != ArmErrorStatus.Normal and not self._error_flag[device_id]:
                self._error_flag[device_id] = True
                self._logger.error("dev%d is error, reason: %s", device_id, error_status.name)
