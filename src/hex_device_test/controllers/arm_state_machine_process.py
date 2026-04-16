import threading
import time
from typing import List,Optional

from ..managers.Coordinator import ArmCoordinatorStatus
from ..statuses.ArmProcessIPC import ArmCommChannel
from ..statuses.ArmStatus import ArmControllerStatus, ArmCoordinatorStatus, ArmErrorStatus,ArmCmdStatus
from  ..controllers.ErrorChecker import ArmErrorChecker
from ..controllers.TrajectoryController import ReturnHomeController
from hex_device import Arm, CommandType, Hands

class ArmCoordinatorProcessStateMachine:
    """
    协调器状态机 - 6状态管理
    """
    
    STATE_TRANSITIONS = {
        ArmCoordinatorStatus.Init: [
            ArmCoordinatorStatus.Ready,
            ArmCoordinatorStatus.Error,
            ArmCoordinatorStatus.Stopped
        ],
        ArmCoordinatorStatus.Ready: [
            ArmCoordinatorStatus.Running,
            ArmCoordinatorStatus.Error,
            ArmCoordinatorStatus.Stopped
            # ArmCoordinatorStatus.Exit
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
            ArmCoordinatorStatus.Stopped,
            ArmCoordinatorStatus.Exit
        ],
        ArmCoordinatorStatus.Exit: []
    }
    
    def __init__(self, coordinator):
        self._coordinator = coordinator
        self._state = ArmCoordinatorStatus.Init
        self._state_lock = threading.RLock()
        self._last_state_change = time.time()
        self._auto_send_cmd = False
        
    def transition_to(self, new_state: ArmCoordinatorStatus, reason: str) -> bool:
        """状态转换（带校验）"""
        with self._state_lock:
            if self._state == new_state:
                return True
            
            if new_state not in self.STATE_TRANSITIONS.get(self._state, []):
                print(f"[Coordinator] 非法状态转换: {self._state.name} -> {new_state.name}")
                return False
            
            print(f"[Coordinator] {self._state.name} -> {new_state.name} | reason: {reason}")
            self._state = new_state
            self._auto_send_cmd = True
            self._last_state_change = time.time()
            return True
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
        
        # any_error = any(
        #     status == ArmControllerStatus.Brake 
        #     for status in self._coordinator.get_all_controller_status()
        # )
        any_error,reason = self._coordinator.check_any_device_error()
        
        if all_ready:
            self.transition_to(ArmCoordinatorStatus.Ready, "all device is ready")
        elif any_error:
            self.transition_to(ArmCoordinatorStatus.Error, reason)
    
    def _handle_ready(self) -> None:
        """
        准备状态处理
        - 事件：设备上报状态为Ready
        - 转移：所有设备running且设备为running命令 → Running
        - 转移：设备Error → Error
        """
        
        # 检查设备错误
        err, reason = self._coordinator.check_any_device_error()
        if err:
            self.transition_to(ArmCoordinatorStatus.Error, reason)
            
        # check cmd
        # stopped -> exit
        if self._coordinator.has_pending_command(ArmCmdStatus.STOPPED):
            all_ready = all(
                status == ArmControllerStatus.Stopped
                for status in self._coordinator.get_all_controller_status()
            )
            if all_ready:
                self.transition_to(ArmCoordinatorStatus.Stopped, "stop")

        # 发布running
        if self._auto_send_cmd:
            self._auto_send_cmd = False
            self._coordinator.publish_command(ArmCmdStatus.RUN.value)
        

        # 检查是否有RUN命令被发布
        if self._coordinator.has_pending_command(ArmCmdStatus.RUN):
            all_ready = all(
                status == ArmControllerStatus.Running
                for status in self._coordinator.get_all_controller_status()
            )
            if all_ready:
                self.transition_to(ArmCoordinatorStatus.Running, "开始运行")
    
    def _handle_running(self) -> None:
        """
        运行状态处理
        - 过程：执行运动
        - 转移1：设备上报Error → Error
        - 转移2：协调器发布STOPPED命令 → Stopped
        """
        # 检查设备错误
        err, reason = self._coordinator.check_any_device_error()
        if err:
            self.transition_to(ArmCoordinatorStatus.Error, reason)
        
        # 更新哪台设备异常
        
        # stopped的转移暂时由外部执行
        # # 检查STOPPED命令
        # if self._coordinator.has_pending_command(ArmCmdStatus.STOPPED):
        #     self.transition_to(ArmCoordinatorStatus.Stopped, "停止命令")
        #     return
        
        # # 检查BRAKE命令
        # if self._coordinator.has_pending_command(ArmCmdStatus.BRAKE):
        #     self.transition_to(ArmCoordinatorStatus.Error, "紧急停止")
        #     return
    
        # send command
        # if self._auto_send_cmd:
        #     self._auto_send_cmd = False
        #     self._coordinator.publish_command(ArmCmdStatus.RUN.value)      

    def _handle_stopped(self) -> None:
        """
        停止中状态处理
        - 转移1：设备均上报Exit或Ready → Ready（循环）/ Exit（结束）
        - 转移2：设备上报Error → Error
        """
        
        # 执行
        
        # check 
        statuses = self._coordinator.get_all_controller_status()
        
        all_Exit = all(
            # s in [ArmControllerStatus.Exit, ArmControllerStatus.Ready]
            s == ArmControllerStatus.Exit
            for s in statuses
        )
        
        any_error = any(
            s == ArmControllerStatus.Brake for s in statuses
        )
        
        if any_error:
            self.transition_to(ArmCoordinatorStatus.Error, "停止过程中被刹车")
        elif all_Exit:
            # 根据命令决定是循环还是退出
            if self._coordinator.has_pending_command(ArmCmdStatus.IDLE):
                self.transition_to(ArmCoordinatorStatus.Ready, "准备下一轮")
            else:
                self.transition_to(ArmCoordinatorStatus.Exit, "完成退出")
        
        # send command
        if self._auto_send_cmd:
            self._auto_send_cmd = False
            self._coordinator.publish_command(ArmCmdStatus.STOPPED.value)
    
    def _handle_error(self) -> None:
        """
        Error状态处理
        - 转移1：协调器命令Exit → Exit
        - 转移2：协调器命令IDLE → Init（恢复）
        - 转移3：协调器命令ZERO_STOPPED → （接口预留）
        
        注：设备恢复接口仅定义，不具体实现
        """
        # if self._coordinator.has_pending_command(ArmCmdStatus.IDLE):
        #     # 恢复运行
        #     self.transition_to(ArmCoordinatorStatus.Init, "恢复运行")
        # elif self._coordinator.has_pending_command(ArmCmdStatus.STOPPED):
        #     # 退出
        #     self.transition_to(ArmCoordinatorStatus.Exit, "异常退出")
            
        if self._coordinator.has_pending_command(ArmCmdStatus.STOPPED):
            # 退出
            self.transition_to(ArmCoordinatorStatus.Exit, "异常退出")
        # ZERO_STOPPED: 预留接口，不处理

        # send command
        if self._auto_send_cmd:
            self._auto_send_cmd = False
            # self._coordinator.publish_command(ArmCmdStatus.BRAKE.value)
            # 正常机械臂刹车
            errors = self._coordinator.get_error_flag()
            for idx, check in enumerate(errors):
                if not check:
                    self._coordinator.publish_dev_command(idx,ArmCmdStatus.BRAKE.value)
            
        
    def _handle_exit(self) -> None:
        """
        退出状态处理
        """
        # self._coordinator._do_cleanup()
        
        # 暂时由外部控制退出clean，状态机转到这里可以停了
        pass
    

        
    def step(self) -> None:
        """
        状态机步进
        """
        handler_map = {
            ArmCoordinatorStatus.Init: self._handle_init,
            ArmCoordinatorStatus.Ready: self._handle_ready,
            ArmCoordinatorStatus.Running: self._handle_running,
            ArmCoordinatorStatus.Stopped: self._handle_stopped,
            ArmCoordinatorStatus.Error: self._handle_error,
            ArmCoordinatorStatus.Exit: self._handle_exit,
        }
        handler = handler_map.get(self._state)
        if handler:
            handler()

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
            ArmControllerStatus.Brake,
            ArmControllerStatus.Running,
            ArmControllerStatus.Stopped,
            ArmControllerStatus.Exit
            
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
    
    def __init__(self, id, arm_ipc: ArmCommChannel):
        self._arm_ipc = arm_ipc
        self._state = ArmControllerStatus.Init
        self._last_reported_state = None
        self._first_start = True
        self._stopped_time = None
        self._dev_id = id
        # ============== return home =================
        self._return_home_controller:Optional[ReturnHomeController] = None
        self._home_position = [0.0, -1.5, 3.00, 0.0, 0.0, 0.0]
        self._return_home_duration = 10.0
    
    def transition(self, new_state: ArmControllerStatus, reason: str) -> bool:
        """状态转换并更新共享内存"""
        if new_state == self._state:
            return True
        
        if new_state not in self.STATE_TRANSITIONS.get(self._state, []):
            print(f"[Dev{self._dev_id}] 非法状态转换: {self._state.name} -> {new_state.name}")
            return False
        # if new_state == ArmControllerStatus.error:
        #     print(f"[dev] err: {reason}")
        print(f"[Dev{self._dev_id}] {self._state.name} -> {new_state.name} | {reason}")
        self._state = new_state
        
        # 更新共享内存（仅在状态变化时）
        self._arm_ipc.set_controller_status(new_state.value)
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
        if self._first_start :
            device.start()
            self._first_start = False    
            
        if self._return_home_controller is None:
            current_pos = device.get_motor_positions()
            if current_pos is None:
                return
            self._return_home_controller = ReturnHomeController(
                start_position=current_pos,
                home_position=self._home_position,
                duration=self._return_home_duration
            )
        
        target_position, reached_home = self._return_home_controller.get_target_position()
        if target_position is not None:
            device.motor_command(CommandType.POSITION, target_position.tolist())    
        
        # Smooth and check Home
        if reached_home:
            self._return_home_controller = None
            self.transition(ArmControllerStatus.Ready, "activate Home")
            return
        
        # 检查Brake命令
        if self._check_cmd(ArmCmdStatus.BRAKE):
            self._return_home_controller = None
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
            self.transition(ArmControllerStatus.Exit, "接收到STOPPED命令")
        
        # 发送空命令，防止apiTimeout
    
    def handle_running(self, device: Arm, target_position:Optional[List[float]]) -> None:
        """
        Running状态
        - 执行：执行轨迹
        - 转移1：发布STOPPED命令 → Stopped
        - 转移2：接收到BRAKE或发生error → Brake
        """
        # running trajectory
        # if trajectory:
        #     target = trajectory.get_current_target()
        #     device.motor_command(CommandType.POSITION, target.tolist())
        
        target = target_position
        if target is not None and hasattr(target,"tolist"):
            device.motor_command(CommandType.POSITION, target.tolist())
        
        # check cmd
        if self._check_cmd(ArmCmdStatus.STOPPED):
            self.transition(ArmControllerStatus.Stopped, "接收到STOPPED命令")
            return
        
        if self._check_cmd(ArmCmdStatus.BRAKE):
            self.transition(ArmControllerStatus.Brake, "接收到BRAKE命令")
            return
        
        # # check err
        # has_error, errors,reasons = ArmErrorChecker.check_device(device)
        # if has_error:
        #     self._report_errors(errors)
        #     self.transition(ArmControllerStatus.Brake, f"检测到异常: {reasons}")
    
    def handle_stopped(self, device: Arm, last_cmd_position:Optional[List[float]]) -> None:
        """
        Stopped状态
        - 执行：返回home
        - 转移1：接收到BRAKE或error → Brake
        - 转移2：返回home完成且命令为IDLE → Ready
        - 转移3：返回home完成且命令为STOPPED → Exit
        """
        # 返回home
        if self._return_home_controller is None:
            # 获取trajectory的最后命令位置，不是当前实际位置
            last_cmd_position = last_cmd_position
            if last_cmd_position is None:
                # 如果没有trajectory，使用当前实际位置
                last_cmd_position = device.get_motor_positions()
            if hasattr(last_cmd_position,"tolist"):
                last_cmd_position = last_cmd_position.tolist()
            self._return_home_controller = ReturnHomeController(
                start_position=last_cmd_position,
                home_position=self._home_position,
                duration=self._return_home_duration/2
            )
            self._stopped_time = time.process_time()

        # 获取平滑归位目标位置
        target_position, reached_home = self._return_home_controller.get_target_position()
        if hasattr(target_position,"tolist"):
            device.motor_command(CommandType.POSITION, target_position.tolist())
        
        # # check err
        # has_error, errors,reasons = ArmErrorChecker.check_device(device)
        # if has_error:
        #     self._return_home_controller = None
        #     self._report_errors(errors)
        #     self.transition(ArmControllerStatus.Brake, f"停止中异常: {errors,reasons}")
        #     return
        
        # check home
        if reached_home:
            self._return_home_controller = None
            if self._check_cmd(ArmCmdStatus.STOPPED):
                self.transition(ArmControllerStatus.Exit, "完成退出")
            # elif self._check_cmd(ArmCmdStatus.IDLE):
            #     pass
        
        # check time out 
        if time.process_time() - self._stopped_time > 8.0:
            self.transition(ArmControllerStatus.Exit, "stopped time out")

            
    def handle_brake(self, device: Arm) -> None:
        """
        Brake状态
        - 执行：锁定当前位置
        - 转移：命令为STOPPED → Exit
        """
        # 锁定位置
        device.motor_command(CommandType.BRAKE, [True] * device.motor_count)
        
        if self._check_cmd(ArmCmdStatus.STOPPED):
            self.transition(ArmControllerStatus.Exit, "brake to stopped")
        
    
    def handle_exit(self) -> None:
        """
        Exit状态
        - 执行：状态上报完成标记
        - 退出循环
        """
        # 标记完成
        self._arm_ipc.set_controller_status(ArmControllerStatus.Exit.value)
    
    def _check_cmd(self, cmd: ArmCmdStatus) -> bool:
        """检查当前命令"""
        return self._arm_ipc.get_cmd_status() == cmd.value
    
    # def _report_errors(self, errors: List[ArmErrorChecker]) -> None:
    #     """报告错误到共享内存"""
    #     # 根据具体的error report
    #     # self._arm_ipc.error_status.value = ArmErrorStatus.Error.value
    #     error_code = min(errors, key=lambda x: x.value)
    #     self._arm_ipc.error_status.value = error_code.value
    #     # 记录详细错误信息
    #     # for error in errors:
    #     #     timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    #     #     # self._arm_ipc.runtime_errors[timestamp] = error