from copy import deepcopy
import multiprocessing
from multiprocessing import Manager
from typing import Dict,Optional

from .ArmStatus import ArmErrorStatus,ArmCmdStatus,ArmControllerStatus

# multiprocessing.set_start_method('forkserver', force=True)

class ArmCommChannel:
    
    def __init__(self):
        """single device struct"""
        self.error_status = multiprocessing.Value("i",0) #error_status: main process read only
        
        self.controller_status = multiprocessing.Value("i",0) # state_machine:  main process read only
        
        # cmd做修改，通过pipe发，共享内存读 
        self.cmd_status = multiprocessing.Value("i",0) # main process read only
        
        self.cmd_recv_pipe,self.cmd_send_pipe = multiprocessing.Pipe(duplex=False)
        
        # self.manager = Manager()
        
    def cleanup(self):
        if hasattr(self,"manager"):
            self.manager.shutdown()
            
class ArmCommChannelManager:
    def __init__(self):
        # pass
        self._ipc_dict: Dict[int,ArmCommChannel] = {}
    
    def create_arm_ipc(self,id):
        if id not in self._ipc_dict:
            shm = ArmCommChannel()
            self._ipc_dict[id] = shm
            return shm
    
    def get_device_ipc(self, device_id: int) -> ArmCommChannel:
        return self._ipc_dict.get(device_id)
    
    def get_ipc_dict(self) -> Dict[int, ArmCommChannel]:
        return self._ipc_dict
    
    def remove_shared_memory(self, device_id: int):
        """移除指定设备的共享内存"""
        if device_id in self._ipc_dict:
            self._ipc_dict[device_id].cleanup()
            del self._ipc_dict[device_id]
    
    def cleanup_all(self):
        """清理所有共享内存"""
        for shm in self._ipc_dict.values():
            shm.cleanup()
        self._ipc_dict.clear()
        
    # def all_pipe_