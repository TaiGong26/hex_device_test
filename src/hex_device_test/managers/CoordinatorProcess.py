from typing import Optional, List,Dict
import threading
import time
from enum import Enum
import multiprocessing as mp
from collections import deque


from .BaseCoordinator import BaseCoordinator
from ..controllers.ArmControllerProcess import ArmControllerMp as Controller
from ..statuses.ArmStatus import ArmCoordinatorStatus,ArmControllerStatus,ArmCoordinatorProcessStateMachine,ArmErrorStatus
from ..statuses.SharedMemory import ArmProcessCommunication,ArmProcessCommunicationManager

class ArmCoordinator(BaseCoordinator):
    
    def __init__(self, device_ws_url_list: Optional[List[dict]] = None, enable_kcp: bool = False, arm_config: Optional[dict] = None, waypoints: Optional[List[dict]] = None, enable_view: bool = False):
        super().__init__()
        
        self._task = None
        self._enable_kcp = enable_kcp
        self._waypoints = waypoints
        self._enable_view = enable_view
        
        # 
        # self._status_queue = mp.Queue(maxsize=50)
        
        # 共享内存管理
        self._shared_memory: Dict[int, ArmProcessCommunication]
        
        # 状态机
        self._state_machine: ArmCoordinatorProcessStateMachine
        
        # 子进程管理
        self._controllers_list: List[Controller]
        
        # 运行时数据
        self._device_states: Dict[int, ArmControllerStatus]
        self._error_states: Dict[int, ArmErrorStatus]
        
        
        
        
        self.start(device_ws_url_list, enable_kcp, arm_config)
    
    def start(self, device_ws_url_list, enable_kcp, arm_config):
        
        if device_ws_url_list is None:
            print("device ip list is None")
            return False
        
        shm_manager = ArmProcessCommunicationManager()
        
        # create controllers
        for idx, ip in enumerate(device_ws_url_list):
            device_shm = shm_manager.create_shared_memory(idx)
            self._shared_memory[idx] = device_shm
            
            controller = Controller(
                ws_url=ip,
                local_port=0,
                enable_kcp=enable_kcp,
                task_loop_hz=100,
                device_id=idx
            )
            controller.set_shared_memory(device_shm)
            self._controllers_list.append(controller)
        
        # event init
        print(f"controllers {len(self._controllers_list)}")

        # setting controllers
        for controller in self._controllers_list:
            controller.set_arm_config(arm_config)
            controller.set_waypoints(self._waypoints)
            controller.set_view(self._enable_view)
            # controller.set_status_callback(self._controller_status_changed)
        
        for controller in self._controllers_list:
            controller.start()
            
        self._task: threading.Thread = threading.Thread(target=self._task_loop)
        self._task.start()
        print(f"task  start")
    
    
    def shutdown(self):
        # from threading to stop controllers
        crl_shutdown_queue = deque()
        with self.controller_lock:
            for controller in self._controllers_list:
                t = threading.Thread(target=controller.shutdown)
                t.start()
                crl_shutdown_queue.append(t)      
        
        for t in crl_shutdown_queue:
            t.join(timeout=30.0)
            
        if self._task:
            self._task.join(timeout=0.1)
            self._task = None
            
        self._stop_event.set()
        print("-------------------------------- Process shutdown ----------------------------------")
    
    
    
    # ============ command ==============
    def publish_command(self,cmd:Optional[str] = None):
        try:
            for controller in self._controllers_list:
                controller.set_current_cmd(cmd)
            
        except Exception as e:
            print(e)
    
    
    # ============ 状态检查 ==============
    def get_controller_status(self) -> List[ArmControllerStatus]:
        return
    
    
    def check_any_device_error(self) -> List[ArmControllerStatus]:
        return
    
    def has_pending_command(self) -> List[ArmControllerStatus]:
        return 
    
    def _scan_device_states(self) -> None:
        pass
    
    
    
    # ============ callback ==============
    
    def all_brake_command(self):
        pass

    def all_home_command(self):
        pass
    
    
    
    # def _task_loop(self):
    #     try:
    #         task_sleep = 0.05
    #         while self._stop_event.is_set() == False: # condition: controllers are running
                
    #             pass
    #     except Exception as e:
    #         print(f"Err task loop: {e}")
            
            
    # 
    def _task_loop(self) -> None:
        """协调器主循环 - 20Hz"""
        try:
            while not self._stop_event.is_set():
                # 1. 扫描设备状态共享内存
                self._scan_device_states()
                
                # 2. 状态机步进  没实现
                self._state_machine.step()
                
                # 3. 检查全局一致性  没实现
                self._check_consistency()
                
                time.sleep(0.05)  # 20Hz
        except Exception as e:
            print(f"[Coordinator] 主循环异常: {e}")

    def _scan_device_states(self) -> None:
        """扫描所有子进程的共享内存状态"""
        for device_id, shm in self._shared_memory.items():
            error_status = ArmErrorStatus(shm.error_status.value)
            controller_status = ArmControllerStatus(shm.controller_status.value)
            
            # 更新本地缓存
            self._device_states[device_id] = controller_status
            self._error_states[device_id] = error_status
            
            # 检测状态变化并记录
            # if controller_status != self._last_device_states.get(device_id):
            #     print(f"[Coordinator] dev{device_id}状态: {controller_status.name}")
            #     self._last_device_states[device_id] = controller_status
            
            # check error
            if error_status != ArmErrorStatus.Normal:
                print(f"[dev{device_id}] is error")
                self._state_machine.transition_to(ArmCoordinatorStatus.Error,"get device error status")