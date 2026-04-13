from enum import Enum
import threading
import time
from typing import List

from ..statuses.SharedMemory import ArmProcessSharedMemory
from  ..controllers.ErrorChecker import ArmErrorChecker
from ..managers.CoordinatorProcess import ArmCoordinator
from hex_device import Arm, CommandType, Hands


class ArmErrorStatus(Enum):
    Normal      = 0
    Warning     = 1
    Error       = 2

class ArmCmdStatus(Enum):
    IDLE = 0
    RUN = 1
    BARKE =2 
    STOPPED =3
    ZERO_STOPPED = 4

class ArmCoordinatorStatus(Enum):
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    Stopped = 4
    Error = 5
    Exit = 6

class ArmControllerStatus(Enum):
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    Brake = 4
    Stopped = 5
    Exit = 6

class ArmCoordinatorProcessStateMachine:
    """
    协调器状态机 - 6状态管理
    """
    
    STATE_TRANSITIONS = {
        ArmCoordinatorStatus.Init: [
            ArmCoordinatorStatus.Ready,
            ArmCoordinatorStatus.Error
        ],
        ArmCoordinatorStatus.Ready: [
            ArmCoordinatorStatus.Running,
            ArmCoordinatorStatus.Error,
            ArmCoordinatorStatus.Exit
        ],
        ArmCoordinatorStatus.Running: [
            ArmCoordinatorStatus.Stopped,
            ArmCoordinatorStatus.Error
        ],
        ArmCoordinatorStatus.Stopped: [
            ArmCoordinatorStatus.Ready,
            ArmCoordinatorStatus.Exit,
            ArmCoordinatorStatus.Error
        ],
        ArmCoordinatorStatus.Error: [
            ArmCoordinatorStatus.Init,
            ArmCoordinatorStatus.Exit
        ],
        ArmCoordinatorStatus.Exit: []
    }
    
    def __init__(self, coordinator: ArmCoordinator):
        self._coordinator: ArmCoordinator = coordinator
        self._state = ArmCoordinatorStatus.Init
        self._state_lock = threading.RLock()
        self._last_state_change = time.time()
    
    # ========== 状态处理函数 ==========
    
    def _handle_init(self) -> None:
        """
        初始化状态处理
        - 过程：创建分控器
        - 过程：等待设备上报状态为Init
        - 转移1：所有设备上报Ready → Ready
        - 转移2：任一设备上报Error → Error
        """
        all_ready = all(
            status == ArmControllerStatus.Ready 
            for status in self._coordinator.get_all_controller_status()
        )
        
        any_error = any(
            status == ArmControllerStatus.Brake 
            for status in self._coordinator.get_all_controller_status()
        )
        
        if all_ready:
            self.transition_to(ArmCoordinatorStatus.Ready, "所有设备准备就绪")
        elif any_error:
            self.transition_to(ArmCoordinatorStatus.Error, "设备初始化异常")
    
    def _handle_ready(self) -> None:
        """
        准备状态处理
        - 事件：设备上报状态为Ready
        - 转移：所有设备Ready且协调器发布RUN命令 → Running
        - 转移：设备Error → Error
        """
        # 检查是否有RUN命令被发布
        if self._coordinator.has_pending_command(ArmCmdStatus.RUN):
            all_ready = all(
                status == ArmControllerStatus.Ready
                for status in self._coordinator.get_all_controller_status()
            )
            if all_ready:
                self.transition_to(ArmCoordinatorStatus.Running, "开始运行")
        
        # 检查设备错误
        if self._coordinator.check_any_device_error():
            self.transition_to(ArmCoordinatorStatus.Error, "设备异常")
    
    def _handle_running(self) -> None:
        """
        运行状态处理
        - 过程：执行运动
        - 转移1：设备上报Error → Error
        - 转移2：协调器发布STOPPED命令 → Stopped
        """
        # 检查STOPPED命令
        if self._coordinator.has_pending_command(ArmCmdStatus.STOPPED):
            self.transition_to(ArmCoordinatorStatus.Stopped, "停止命令")
            return
        
        # 检查BARKE命令
        if self._coordinator.has_pending_command(ArmCmdStatus.BARKE):
            self.transition_to(ArmCoordinatorStatus.Error, "紧急停止")
            return
        
        # 检查设备错误
        if self._coordinator.check_any_device_error():
            self.transition_to(ArmCoordinatorStatus.Error, "运行中异常")
    
    def _handle_stopped(self) -> None:
        """
        停止中状态处理
        - 转移1：设备均上报Exit或Ready → Ready（循环）/ Exit（结束）
        - 转移2：设备上报Error → Error
        """
        statuses = self._coordinator.get_all_controller_status()
        
        all_stopped = all(
            s in [ArmControllerStatus.Stopped, ArmControllerStatus.Ready]
            for s in statuses
        )
        
        any_error = any(
            s == ArmControllerStatus.Brake for s in statuses
        )
        
        if any_error:
            self.transition_to(ArmCoordinatorStatus.Error, "停止过程中异常")
        elif all_stopped:
            # 根据命令决定是循环还是退出
            if self._coordinator.has_pending_command(ArmCmdStatus.IDLE):
                self.transition_to(ArmCoordinatorStatus.Ready, "准备下一轮")
            else:
                self.transition_to(ArmCoordinatorStatus.Exit, "完成退出")
    
    def _handle_error(self) -> None:
        """
        Error状态处理
        - 转移1：协调器命令Exit → Exit
        - 转移2：协调器命令IDLE → Init（恢复）
        - 转移3：协调器命令ZERO_STOPPED → （接口预留）
        
        注：设备恢复接口仅定义，不具体实现
        """
        if self._coordinator.has_pending_command(ArmCmdStatus.IDLE):
            # 恢复运行
            self.transition_to(ArmCoordinatorStatus.Init, "恢复运行")
        elif self._coordinator.has_pending_command(ArmCmdStatus.STOPPED):
            # 退出
            self.transition_to(ArmCoordinatorStatus.Exit, "异常退出")
        # ZERO_STOPPED: 预留接口，不处理
    
    def _handle_exit(self) -> None:
        """
        退出状态处理
        - 执行清理
        - 终止所有子进程
        """
        self._coordinator._do_cleanup()
    
    def transition_to(self, new_state: ArmCoordinatorStatus, reason: str) -> bool:
        """状态转换（带校验）"""
        with self._state_lock:
            if new_state not in self.STATE_TRANSITIONS.get(self._state, []):
                print(f"[Coordinator] 非法状态转换: {self._state.name} -> {new_state.name}")
                return False
            
            print(f"[Coordinator] {self._state.name} -> {new_state.name} | 原因: {reason}")
            self._state = new_state
            self._last_state_change = time.time()
            return True


class ArmControllerProcessStateMachine:
    """
    子进程状态机 - 5状态管理（不含Disconnected）
    Disconnected在进程启动前由协调器管理
    """
    
    STATE_TRANSITIONS = {
        ArmControllerStatus.Init: [
            ArmControllerStatus.Ready,
            ArmControllerStatus.Brake
        ],
        ArmControllerStatus.Ready: [
            ArmControllerStatus.Running,
            ArmControllerStatus.Stopped
        ],
        ArmControllerStatus.Running: [
            ArmControllerStatus.Stopped,
            ArmControllerStatus.Brake
        ],
        ArmControllerStatus.Stopped: [
            ArmControllerStatus.Ready,
            ArmControllerStatus.Brake,
            ArmControllerStatus.Exit
        ],
        ArmControllerStatus.Brake: [
            ArmControllerStatus.Exit
        ],
        ArmControllerStatus.Exit: []
    }
    
    def __init__(self, shared_memory: ArmProcessSharedMemory):
        self._shared_memory = shared_memory
        self._state = ArmControllerStatus.Init
        self._last_reported_state = None
    
    def transition(self, new_state: ArmControllerStatus, reason: str) -> bool:
        """状态转换并更新共享内存"""
        if new_state not in self.STATE_TRANSITIONS.get(self._state, []):
            print(f"[Device] 非法状态转换: {self._state.name} -> {new_state.name}")
            return False
        
        print(f"[Device] {self._state.name} -> {new_state.name} | {reason}")
        self._state = new_state
        
        # 更新共享内存（仅在状态变化时）
        self._shared_memory.controller_status.value = new_state.value
        return True
    
    def get_state(self) -> ArmControllerStatus:
        return self._state
    
    # ========== 状态处理函数 ==========
    
    def handle_init(self, device: Arm) -> None:
        """
        初始化状态
        - 执行：上线之后移动到home
        - 转移：到达home → Ready
        - 转移：Error/Brake命令 → Brake
        """
        # 启动设备
        device.start()
        
        # 移动到home位置
        home_position = [0.0, -1.5, 3.00, 0.0, 0.0, 0.0]
        device.motor_command(CommandType.POSITION, home_position)
        
        # 检查是否到达home
        if self._is_at_home(device):
            self.transition(ArmControllerStatus.Ready, "activate Home")
        
        # 检查Brake命令
        if self._check_cmd(ArmCmdStatus.BARKE):
            self.transition(ArmControllerStatus.Brake, "Command Brake")
    
    def handle_ready(self) -> None:
        """
        Ready状态
        - 执行：pass
        - 转移：命令为RUN → Running
        - 转移：命令为STOPPED → Stopped
        """
        if self._check_cmd(ArmCmdStatus.RUN):
            self.transition(ArmControllerStatus.Running, "接收到RUN命令")
        elif self._check_cmd(ArmCmdStatus.STOPPED):
            self.transition(ArmControllerStatus.Stopped, "接收到STOPPED命令")
    
    def handle_running(self, device: Arm, trajectory) -> None:
        """
        Running状态
        - 执行：执行轨迹
        - 转移1：发布STOPPED命令 → Stopped
        - 转移2：接收到BARKE或发生error → Brake
        """
        # 执行轨迹
        if trajectory:
            target = trajectory.get_current_target()
            device.motor_command(CommandType.POSITION, target.tolist())
        
        # 检查命令
        if self._check_cmd(ArmCmdStatus.STOPPED):
            self.transition(ArmControllerStatus.Stopped, "接收到STOPPED命令")
            return
        
        if self._check_cmd(ArmCmdStatus.BARKE):
            self.transition(ArmControllerStatus.Brake, "接收到BARKE命令")
            return
        
        # 检查异常
        has_error, errors = ArmErrorChecker.check_device(device)
        if has_error:
            self._report_errors(errors)
            self.transition(ArmControllerStatus.Brake, f"检测到异常: {errors}")
    
    def handle_stopped(self, device: Arm) -> None:
        """
        Stopped状态
        - 执行：返回home
        - 转移1：接收到BARKE或error → Brake
        - 转移2：返回home完成且命令为IDLE → Ready
        - 转移3：返回home完成且命令为STOPPED → Exit
        """
        # 返回home
        home_position = [0.0, -1.5, 3.00, 0.0, 0.0, 0.0]
        device.motor_command(CommandType.POSITION, home_position)
        
        # 检查异常
        has_error, errors = ArmErrorChecker.check_device(device)
        if has_error:
            self._report_errors(errors)
            self.transition(ArmControllerStatus.Brake, f"停止中异常: {errors}")
            return
        
        # 检查是否到达home
        if self._is_at_home(device):
            if self._check_cmd(ArmCmdStatus.IDLE):
                self.transition(ArmControllerStatus.Ready, "准备下一轮")
            elif self._check_cmd(ArmCmdStatus.STOPPED):
                self.transition(ArmControllerStatus.Exit, "完成退出")
    
    def handle_brake(self, device: Arm) -> None:
        """
        Brake状态
        - 执行：锁定当前位置
        - 转移：命令为STOPPED → Exit
        """
        # 锁定位置
        current_pos = device.get_motor_positions()
        device.motor_command(CommandType.POSITION, current_pos)
        
        # 检查退出命令
        if self._check_cmd(ArmCmdStatus.STOPPED):
            self.transition(ArmControllerStatus.Exit, "异常退出")
    
    def handle_exit(self) -> None:
        """
        Exit状态
        - 执行：状态上报完成标记
        - 退出循环
        """
        # 标记完成
        self._shared_memory.controller_status.value = ArmControllerStatus.Exit.value
    
    def _check_cmd(self, cmd: ArmCmdStatus) -> bool:
        """检查当前命令"""
        return self._shared_memory.cmd_status.value == cmd.value
    
    def _report_errors(self, errors: List[str]) -> None:
        """报告错误到共享内存"""
        self._shared_memory.error_status.value = ArmErrorStatus.Error.value
        # 记录详细错误信息
        for error in errors:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            self._shared_memory.runtime_errors[timestamp] = error