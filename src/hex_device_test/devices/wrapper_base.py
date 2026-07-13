"""WrapperBase — 设备操作抽象接口

设计意图：
Controller 代码不直接调用 hex_device 或 hex_driver_robot，通过此抽象基类
收拢所有设备操作。切换底层驱动时只需换一个后端实现类。
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

import numpy as np


class WrapperBase(ABC):
    """设备操作抽象基类

    私有属性（子类实现）：
        _api       — 底层 API 对象（HexDeviceApi / HexDriver）
        _robot     — 设备实例（Arm）
        _robot_type: int
        _dof: int  — 电机数量

    生命周期：
        start() → [discover_device] → [command/read]* → shutdown()
    """

    # ── 生命周期 ──

    @abstractmethod
    def start(self, ws_url: str, enable_kcp: bool = False) -> bool:
        """启动底层连接（创建 _api）"""

    @abstractmethod
    def shutdown(self):
        """释放所有资源"""

    # ── 设备发现 / 配置 ──

    @abstractmethod
    def discover_device(self, arm_config: Optional[dict] = None) -> bool:
        """发现或创建 Arm 设备实例，设置 _robot / _robot_type / _dof"""

    # ── 属性 ──

    @property
    @abstractmethod
    def motor_count(self) -> int:
        """电机数量（_dof）"""

    @property
    @abstractmethod
    def robot_type(self) -> int:
        """机器人类型枚举值"""

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
    def motor_command_position(self, target: Any) -> None:
        """发送位置控制命令"""

    @abstractmethod
    def motor_command_brake(self) -> None:
        """发送刹车命令"""

    @abstractmethod
    def motor_start(self) -> None:
        """启动电机"""

    @abstractmethod
    def reset_last_command_time(self) -> None:
        """重置最后命令时间戳"""