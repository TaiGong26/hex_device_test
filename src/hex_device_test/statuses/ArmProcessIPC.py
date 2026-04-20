from copy import deepcopy
import multiprocessing
from multiprocessing import Manager
from typing import Dict,Optional

from .ArmStatus import ArmErrorStatus,ArmCmdStatus,ArmControllerStatus

# multiprocessing.set_start_method('forkserver', force=True)

class ArmCommChannel:
    
    def __init__(self):
        """single device struct"""
        self._error_status = multiprocessing.Value("i",0) #error_status: main process read only
        
        self._controller_status = multiprocessing.Value("i",0) # state_machine:  main process read only
        
        # cmd做修改，通过pipe发，共享内存读 
        self._cmd_status = multiprocessing.Value("i",0) # main process read only
        
        self._lock = multiprocessing.Lock()
        
        self.cmd_recv_pipe,self.cmd_send_pipe = multiprocessing.Pipe(duplex=False)
        
        # self.manager = Manager()
    
    def get_error_status(self):
        with self._lock:
            return self._error_status.value
    
    def get_controller_status(self):
        with self._lock:
            return self._controller_status.value
    
    def get_cmd_status(self):
        with self._lock:
            return self._cmd_status.value

    def set_error_status(self, state):
        if isinstance(state,int):
            with self._lock:
                self._error_status.value = state
        else:
            raise ValueError("IPC Set State is Not INT")

    def set_controller_status(self, state):
        if isinstance(state,int):
            with self._lock:
                self._controller_status.value = state
        else:
            raise ValueError("IPC Set State is Not INT")

    def set_cmd_status(self, state):
        if isinstance(state,int):
            with self._lock:
                self._cmd_status.value = state
        else:
            raise ValueError("IPC Set State is Not INT")

    def cleanup(self):
        if hasattr(self,"manager"):
            self.manager.shutdown()
        if self.cmd_send_pipe :
            self.cmd_send_pipe.close()
            
class ArmCommChannelManager:
    def __init__(self):
        # pass
        self._ipc_dict: Dict[int,ArmCommChannel] = {}
    
    def create_arm_ipc(self,id):
        if id not in self._ipc_dict:
            ipc = ArmCommChannel()
            self._ipc_dict[id] = ipc
            return ipc
    
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
        for ipc in self._ipc_dict.values():
            ipc.cleanup()
        self._ipc_dict.clear()
        
    # def all_pipe_