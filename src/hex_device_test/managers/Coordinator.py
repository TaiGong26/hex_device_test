from typing import Optional, List
import threading
import time
from enum import Enum


from .BaseCoordinator import BaseCoordinator
from ..controllers.ArmController import ArmController as Controller
from collections import deque
from ..tools.plotjuggle import senders


class ArmCoordinatorStatus(Enum):
    NoneStatus = 0
    Init = 1
    Ready = 2
    Running = 3
    Stopped = 4
    Error = 5
    Exit = 6


class ArmCoordinator(BaseCoordinator):
    
    def __init__(self, device_ws_url_list: Optional[List[dict]] = None, enable_kcp: bool = False, arm_config: Optional[dict] = None, waypoints: Optional[List[dict]] = None, enable_view: bool = False):
        super().__init__()
        
        self._coordinator_monitor_loop = None
        self._enable_kcp = enable_kcp
        self._waypoints = waypoints
        self._enable_view = enable_view
        # print(f"enable_view: {self._enable_view}")
        
        self.start(device_ws_url_list, enable_kcp, arm_config)
    
    def start(self, device_ws_url_list, enable_kcp, arm_config):
        
        # senders.start()
        
        if device_ws_url_list is None:
            print("device ip list is None")
            return False
        
        # create controllers
        for idx, ip in enumerate(device_ws_url_list):
            controller = Controller(
                ws_url=ip,
                local_port=0,
                enable_kcp=enable_kcp,
                crl_hz=500,
                device_id=idx
            )
            self._controllers_list.append(controller)
        
        # event init
        print(f"controllers {len(self._controllers_list)}")
        # self._send_barrier = threading.Barrier(len(self._controllers_list)+1)
        # self._complete_action_barrier = threading.Barrier(len(self._controllers_list)+1)
        
        # setting controllers
        for controller in self._controllers_list:
            controller.set_arm_config(arm_config)
            controller.set_waypoints(self._waypoints)
            controller.set_view(self._enable_view)
            
            
        
        # # controllers start
        res_crl_start = [False] * len(device_ws_url_list)
        crl_connet_queue = deque()
        
        def connect_wrapper(idx, controller):
          res_crl_start[idx] = controller.start()
          
        
        s =time.time()
        
        for idx, controller in enumerate(self._controllers_list):
            t = threading.Thread(target=connect_wrapper, args=(idx, controller))
            t.start()
            crl_connet_queue.append(t)
        
        print(f"controllers start time: {time.time() - s}")
        for t in crl_connet_queue:
            t.join()
        
        for idx, res in enumerate(res_crl_start):
            if res == False:
                print(f"controller {idx} start failed")
        
        # for controller in self._controllers_list:
        #     controller.start()
        
        # # # coordinator thread start
        # self._coordinator_monitor_loop: threading.Thread = threading.Thread(target=self._monitor_loop)
        # self._coordinator_monitor_loop.start()
    
    
    def shutdown(self):
        # senders.stop()
        
        # from threading to stop controllers
        crl_shutdown_queue = deque()
        with self.controller_lock:
            for controller in self._controllers_list:
                t = threading.Thread(target=controller.shutdown)
                t.start()
                crl_shutdown_queue.append(t)      
        
        for t in crl_shutdown_queue:
            t.join(timeout=3.0)
            
        # coordinator thread stop
        if self._coordinator_monitor_loop:
            self._coordinator_monitor_loop.join(timeout=0.1)
            self._coordinator_monitor_loop = None
            
        self._stop_event.set()
        print("-------------------------------- coordinator shutdown ----------------------------------")

    def _monitor_loop(self):
        
        while not self._stop_event.is_set():
            alive = [c.get_device_id() for c in self._controllers_list if c.is_alive()]
            dead  = list(self.get_dead_threads().keys())
 
            if dead:
                print(f"[Monitor] 存活: {alive}  已退出: {dead}")
 
            self._stop_event.wait(timeout=1.0)
    
    def _task_loop(self):
        
        cmd = None
        while not self._stop_event.is_set(): # condition: controllers are running
            
            # if self.cmd_queue.empty():
            cmd = self.cmd_queue.get()
            
            if cmd:
                
                with self.controller_lock:
                    for controller in self._controllers_list:
                        isok = controller.put_command(cmd)
                        if isok == False:
                            print(f"[Coordinator] device {controller.get_device_id()} put command failed")
                
                # wait for controllers to finish
                # if self._send_barrier:
                #     self._send_barrier.wait()
                #     print("======================= all controllers execute command ============================")
                
                # for controller in self._controllers_list:
                #     is_ok = controller.send_command(cmd)
                #     if not is_ok:
                #         print(f"[Coordinator] device {controller.get_device_id()} send command failed")
                    
                # wait for controllers to finish
                # if self._complete_action_barrier:
                #     self._complete_action_barrier.wait()
                #     print("======================= all controllers complete action ============================")
                
            # time.sleep(0.001)

            # judge status and update status machine. 判断状态并决定是否进行下一次的机械臂变换
        
    # ============ command ==============
    def publish_command(self,cmd:Optional[str] = None):
        try:
            # self._message_bus.publish("Arm_command", cmd)
            
            for controller in self._controllers_list:
                controller.set_current_cmd(cmd)
            
        except Exception as e:
            print(e)
    
    def all_brake_command(self):
        pass

    def all_home_command(self):
        pass