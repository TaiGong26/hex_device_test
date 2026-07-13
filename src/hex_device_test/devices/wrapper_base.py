"""WrapperBase — 设备操作抽象接口

设计意图：
Controller 代码不直接调用 hex_device 或 hex_driver_robot，通过此抽象基类
收拢所有设备操作。切换底层驱动时只需换一个后端实现类。

日志说明：
- wrapper 自身使用 print 输出（前缀 [ArcherY6]），不依赖 logging 模块
- 底层后端（如 BackendHexDevice）可通过 __init__ 传入的 logger 自行记录
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np


from hex_driver_robot.base import HexRobotBaseParams
from hex_driver_robot.tcp_base.generated.public_api_types_pb2 import (
    RobotType as _RobotType,
    SecondaryDeviceType as _SecondaryDeviceType,
)

ARM_ROBOT_TYPE_MAP = {
    "Archer_y6": _RobotType.RtArmArcherY6_H1,
    "Firefly_y6": _RobotType.RtArmFireflyY6_H1,
}

ARM_INFO_MAP = {
    "Archer_y6": {
        "robot_type": _RobotType.RtArmArcherY6_H1,
        "motor_count": 6,
    },
    "Firefly_y6": {
        "robot_type": _RobotType.RtArmFireflyY6_H1,
        "motor_count": 6,
    },
}

GRIP_INFO_MAP = {
    "gp80": {
        "robot_type": _SecondaryDeviceType.SdtHandGp80G1,
        "motor_count": 1,
    },
    "gp100": {
        "robot_type": _SecondaryDeviceType.SdtHandGp100,
        "motor_count": 1,
    },
}

@dataclass
class WrapperParams(HexRobotBaseParams):
    enable_kcp:     bool        = True
    log_level:      str         = "DEBUG"
    grip_type:      str         = "empty"
    robot_name:     str         = "Archer_y6"
    

class WrapperBase(ABC):
    """设备操作抽象基类

    私有属性（子类实现）：
        _api       — 底层 API 对象（HexDeviceApi / HexDriver）
        _robot     — 设备实例（Arm）
        _robot_type: int
        _dof: int  — 电机数量
        _logger    — 可选，传给后端使用（wrapper 自身不用）

    生命周期：
        start() → [discover_device] → [command/read]* → shutdown()
    """

    def __init__(self, logger: Optional[Any] = None):
        self._logger = logger  # 后端使用

    # ── 日志（wrapper 使用 print，不依赖 logging）──

    def _log_info(self, msg: str) -> None:
        print(f"[Arm] {msg}")

    def _log_warn(self, msg: str) -> None:
        print(f"\033[33m[Arm] {msg}\033[0m")

    def _log_err(self, msg: str) -> None:
        print(f"\033[31m[Arm] {msg}\033[0m")

    def _log_debug(self, msg: str) -> None:
        print(f"\033[34m[Arm] {msg}\033[0m")

    # ── 生命周期 ──

    @abstractmethod
    def start(self, ws_url: str, enable_kcp: bool = False) -> bool:
        """启动底层连接（创建 _api）"""

    @abstractmethod
    def shutdown(self):
        """释放所有资源"""

    # ── 属性 ──
    @property
    @abstractmethod
    def raw_device(self) -> Optional[Any]:
        """原始设备对象（_robot）"""

    # ── 状态读取 ──

    @abstractmethod
    def get_motor_positions(self) -> Optional[np.ndarray]:
        """读取所有电机当前角度（弧度）"""

    @abstractmethod
    def get_motor_temperatures(self) -> tuple:
        """(motor_temps, driver_temps)，每个元素 Optional[list]"""

    @abstractmethod
    def get_motor_driver_temperatures(self) -> Optional[list]:
        """驱动温度"""

    @abstractmethod
    def get_motor_error_codes(self) -> Optional[List[Optional[int]]]:
        """电机错误码列表"""

    @abstractmethod
    def get_session_info(self) -> Dict[str, Any]:
        """返回 dict: my_session_id, session_holder, robot_mode, calibrated"""

    # ── 错误检查 ──

    @abstractmethod
    def is_api_exit(self) -> bool:
        """API 是否已退出"""

    @abstractmethod
    def is_websocket_recv_timeout(self) -> bool:
        """WebSocket 接收超时"""

    @abstractmethod
    def get_parking_stop_detail(self):
        """停车详情（hex_driver_robot 返回默认值）"""

    # ── 控制命令 ──

    @abstractmethod
    def motor_command(self, target: Any) -> None:
        """发送位置控制命令"""


    @abstractmethod
    def reset_last_command_time(self) -> None:
        """重置最后命令时间戳"""