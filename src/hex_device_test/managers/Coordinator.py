from typing import Optional, List
import threading
import time


from .BaseCoordinator import BaseCoordinator
from ..controllers.ArmController import ArmController as Controller
from .message_bus import arm_message_bus


class ArmCoordinator(BaseCoordinator):
    
    def __init__(self, device_ws_url_list: Optional[List[dict]] = None, enable_kcp: bool = False, arm_config: Optional[dict] = None, waypoints: Optional[List[dict]] = None):
        super().__init__()
        
        self._coordinator_monitor_loop = None
        self._enable_kcp = enable_kcp
        self._waypoints = waypoints
        
        self.start(device_ws_url_list, enable_kcp, arm_config)

        # self._message_bus = arm_message_bus
    
    def start(self, device_ws_url_list, enable_kcp, arm_config):
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
        self._send_barrier = threading.Barrier(len(self._controllers_list)+1)
        self._complete_action_barrier = threading.Barrier(len(self._controllers_list)+1)
        
        # setting controllers
        for controller in self._controllers_list:
            controller.set_arm_config(arm_config)
            controller.set_waypoints(self._waypoints)
            
        # controllers start
        for controller in self._controllers_list:
            controller.start()
        
        # # # coordinator thread start
        # self._coordinator_monitor_loop: threading.Thread = threading.Thread(target=self._monitor_loop)
        # self._coordinator_monitor_loop.start()
    
    
    def shutdown(self):
        
        # controllers stop
        with self.controller_lock:
            for controller in self._controllers_list:
                controller.shutdown()
            
        # coordinator thread stop
        if self._coordinator_monitor_loop:
            self._coordinator_monitor_loop.join(timeout=0.1)
            self._coordinator_monitor_loop = None
            
        # barrier break
        self._send_barrier.abort()
        self._complete_action_barrier.abort()
        
        
        self._stop_event.set()
        print("--------------------------------coordinator shutdown")

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
                if self._send_barrier:
                    self._send_barrier.wait()
                    print("======================= all controllers execute command ============================")
                
                # for controller in self._controllers_list:
                #     is_ok = controller.send_command(cmd)
                #     if not is_ok:
                #         print(f"[Coordinator] device {controller.get_device_id()} send command failed")
                    
                # wait for controllers to finish
                if self._complete_action_barrier:
                    self._complete_action_barrier.wait()
                    print("======================= all controllers complete action ============================")
                
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