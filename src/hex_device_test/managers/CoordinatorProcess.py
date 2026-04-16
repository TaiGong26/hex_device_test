import traceback
from typing import Optional, List,Dict
import threading
import time
from enum import Enum
import multiprocessing as mp
from collections import deque

# mp.set_start_method('forkserver', force=True)
from .BaseCoordinator import BaseCoordinator
from ..controllers.ArmControllerProcess import ArmControllerMp as Controller
from ..controllers.arm_state_machine_process import ArmCoordinatorProcessStateMachine
from ..statuses.ArmStatus import ArmCmdStatus,ArmCoordinatorStatus,ArmControllerStatus,ArmErrorStatus
from ..statuses.ArmProcessIPC import ArmCommChannel,ArmCommChannelManager
from ..tools.CsvLogger import write_csv

class ArmCoordinator(BaseCoordinator):
    
    def __init__(self, device_ws_url_list: Optional[List[dict]] = None, enable_kcp: bool = False, arm_config: Optional[dict] = None, waypoints: Optional[List[dict]] = None, enable_view: bool = False):
        super().__init__()
        
        self._task = None
        self._enable_kcp = enable_kcp
        self._waypoints = waypoints
        self._enable_view = enable_view
        
        # 进程通信管理
        self._arm_ipc = ArmCommChannelManager()
        self._mp_quque = mp.Queue()
        
        # 状态机
        self._state_machine: ArmCoordinatorProcessStateMachine = ArmCoordinatorProcessStateMachine(self)
        
        # 子进程管理
        self._controllers_list: List[Controller] = []
        
        # 运行时数据
        self._device_states: Dict[int, ArmControllerStatus] = {}
        self._error_states: Dict[int, ArmErrorStatus] = {}
        
        self._error_flag:List = []
        
        self.start(device_ws_url_list, enable_kcp, arm_config)
    
    def start(self, device_ws_url_list, enable_kcp, arm_config):
        
        if device_ws_url_list is None:
            print("device ip list is None")
            return False
        
        # create controllers
        for idx, ip in enumerate(device_ws_url_list):
            device_ipc = self._arm_ipc.create_arm_ipc(idx)
            
            controller = Controller(
                ws_url=ip,
                local_port=0,
                enable_kcp=enable_kcp,
                task_loop_hz=500,
                device_id=idx
            )
            
            controller.set_arm_ipc(device_ipc)
            self._controllers_list.append(controller)
        
        # event init
        print(f"controllers {len(self._controllers_list)}")

        self._error_flag = [False] * len(self._controllers_list)
        # setting controllers
        for controller in self._controllers_list:
            controller.set_arm_config(arm_config)
            controller.set_waypoints(self._waypoints)
            controller.set_view(self._enable_view)
            controller.set_mp_queue(self._mp_quque)
            # controller.set_status_callback(self._controller_status_changed)
        
        for controller in self._controllers_list:
            controller.start()
            
        self._task: threading.Thread = threading.Thread(target=self._task_loop)
        self._task.start()
        
    
    def shutdown(self):
        # 更新状态为stoped
        self._state_machine.transition_to(ArmCoordinatorStatus.Stopped, "shutdown")
        # wait exit status
        stopped_time = 0
        while self._state_machine._state != ArmCoordinatorStatus.Exit:
            time.sleep(0.1)
            stopped_time+=0.1
            if stopped_time ==20 :
                break
        
        # from threading to stop controllers
        crl_shutdown_queue = deque()
        with self.controller_lock:
            for controller in self._controllers_list:
                t = threading.Thread(target=controller.shutdown)
                t.start()
                crl_shutdown_queue.append(t)      
        
        for t in crl_shutdown_queue:
            t.join(timeout=20.0)
            
        if self._task:
            self._task.join(timeout=0.1)
            self._task = None
        
        # 获取队列参数
        # while not self._mp_quque.empty():
        #     info = self._mp_quque.get()
        #     print(info)
        t = time.strftime("%Y-%m-%d %H:%M:%S")
        write_csv(self._mp_quque,f"~/hex_device_log/arm_test_{t}.csv")
        
        # ipc_clean
        self._ipc_clean()
        self._mp_quque.close()
        
        self._stop_event.set()
        print("-------------------------------- Process shutdown ----------------------------------")
    
    def _ipc_clean(self):
        try:
            self._arm_ipc.cleanup_all()
        except Exception as e:
            print(f"[coordinator] IPC clean err: {e}")
    
    # ============ command ==============
    def publish_command(self,cmd:int):
        try:
            ipc_dict = self._arm_ipc.get_ipc_dict()
            for device_id, ipc in ipc_dict.items():
                if not ipc.cmd_send_pipe.closed:
                    if isinstance(cmd,int):
                        ipc.cmd_send_pipe.send(cmd)
                    elif isinstance(cmd,ArmCmdStatus):
                        ipc.cmd_send_pipe.send(cmd.value)
            print(f"[Coordinator] send command: {ArmCmdStatus(cmd).name}")
                    
        except Exception as e:
            print(f"[Coordinator] 命令发送异常: {e}")
    
    def publish_dev_command(self,id:int,cmd:int):
        try:
            ipc = self._arm_ipc.get_device_ipc(id)
            
            if not ipc.cmd_send_pipe.closed:
                    if isinstance(cmd,int):
                        ipc.cmd_send_pipe.send(cmd)
                    elif isinstance(cmd,ArmCmdStatus):
                        ipc.cmd_send_pipe.send(cmd.value)
            
            print(f"[Coordinator] send command to dev{id}: {ArmCmdStatus(cmd).name}")
                    
        except Exception as e:
            print(f"[Coordinator] 命令发送异常: {e}")
    
    # ============ 状态检查 ==============
    def get_error_flag(self):
        return self._error_flag.copy()
    
    def get_all_controller_status(self) -> List[ArmControllerStatus]:
        """获取所有子进程状态"""
        return [
            ArmControllerStatus(ipc.get_controller_status())
            for ipc in self._arm_ipc.get_ipc_dict().values()
        ]

    def check_any_device_error(self) -> bool:
        """检查是否有设备报错"""
        for idx, ipc in self._arm_ipc.get_ipc_dict().items():
            status = ArmErrorStatus(ipc.get_error_status())

            if status.value > ArmErrorStatus.Warning.value:
                return True, f"dev{idx} {status.name}"

        return False, ""

    def has_pending_command(self, cmd: ArmCmdStatus) -> bool:
        """检查设备命令"""
        return any(
            ipc.get_cmd_status() == cmd.value
            for ipc in self._arm_ipc.get_ipc_dict().values()
        )
    
    # ============ callback ==============
    
    # ============ task ==============
    
    def _task_loop(self) -> None:
        """协调器主循环 - 20Hz"""
        try:
            while not self._stop_event.is_set():
                # 1. 扫描设备状态共享内存
                self._scan_device_states()
                
                # 2. 状态机步进
                self._state_machine.step()
            
                time.sleep(0.01)  # 20Hz
        except Exception as e:
            print(f"[Coordinator] 主循环异常: {e}")
            traceback.print_exc()
            

    def _scan_device_states(self) -> None:
        """扫描所有子进程的共享内存状态"""
        ipc_dict = self._arm_ipc.get_ipc_dict()
        
        for device_id, ipc in ipc_dict.items():
            error_status = ArmErrorStatus(ipc.get_error_status())
            controller_status = ArmControllerStatus(ipc.get_controller_status())
            
            # 更新本地缓存
            self._device_states[device_id] = controller_status
            self._error_states[device_id] = error_status
            
            # check error
            if error_status != ArmErrorStatus.Normal and not self._error_flag[device_id]:
                self._error_flag[device_id] = True
                print(f"[dev{device_id}] is error, reason{error_status.name}")
                if self._state_machine._state != ArmCoordinatorStatus.Error:
                    self._state_machine.transition_to(ArmCoordinatorStatus.Error,"get device error status")
