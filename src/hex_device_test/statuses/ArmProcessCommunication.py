import multiprocessing
from multiprocessing import Manager
from typing import Dict,Optional

from .ArmStatus import ArmErrorStatus,ArmCmdStatus,ArmControllerStatus


class ArmCommChannel:
    
    def __init__(self):
        """single device struct"""
        self.error_status = multiprocessing.Value("i",0)
        
        self.controller_status = multiprocessing.Value("i",0)
        
        # cmd做修改，通过pipe发，共享内存读 
        self.cmd_status = multiprocessing.Value("i",0)
        
        self.cmd_send_pipe,self.cmd_recv_pipe = multiprocessing.Pipe(duplex=False)
        
        self.manager = Manager()
        
    def cleanup(self):
        if hasattr(self,"manager"):
            self.manager.shutdown()
            
class ArmCommChannelManager:
    def __init__(self):
        # pass
        self._shared_memories: Dict[int,ArmCommChannel] = {}
    
    def create_shared_memory(self,id):
        if id not in self._shared_memories:
            shm = ArmCommChannel()
            self._shared_memories[id] = shm
    
    def get_shared_memory(self, device_id: int) -> ArmCommChannel:
        """获取指定设备的共享内存"""
        return self._shared_memories.get(device_id)
    
    def get_all_shared_memories(self) -> Dict[int, ArmCommChannel]:
        """获取所有设备的共享内存"""
        return self._shared_memories.copy()
    
    def remove_shared_memory(self, device_id: int):
        """移除指定设备的共享内存"""
        if device_id in self._shared_memories:
            self._shared_memories[device_id].cleanup()
            del self._shared_memories[device_id]
    
    def cleanup_all(self):
        """清理所有共享内存"""
        for shm in self._shared_memories.values():
            shm.cleanup()
        self._shared_memories.clear()
        
    # def all_pipe_