import os
import csv
import time
from typing import List, Dict

from ..statuses.ArmStatus import ArmControllerStatus, ArmErrorStatus

class CsvLogger:
    """
    CSV日志记录器
    - 进程退出时确保数据写入
    """
    
    COLUMNS = [
        'timestamp', 'state', 'error_status',
        'motor_pos_0', 'motor_pos_1', 'motor_pos_2',
        'motor_pos_3', 'motor_pos_4', 'motor_pos_5',
        'target_pos_0', 'target_pos_1', 'target_pos_2',
        'target_pos_3', 'target_pos_4', 'target_pos_5',
        'temp_0', 'temp_1', 'temp_2', 'temp_3', 'temp_4', 'temp_5'
    ]
    
    def __init__(self, robot_type, device_id: int):
        self.device_id = device_id
        self.log_dir = "~/hex_device_logs"
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.file_path = f"{self.log_dir}/{robot_type}_{device_id}_{timestamp}.csv"
        
        self._buffer: List[Dict] = []
        # self._buffer_size = 100
        self._file = None
        self._writer = None
        
        self._init_file()
    
    def _init_file(self) -> None:
        """初始化CSV文件"""
        self._file = open(self.file_path, 'w', newline='')
        self._writer = csv.DictWriter(self._file, fieldnames=self.COLUMNS)
        self._writer.writeheader()
    
    def log(self, state: ArmControllerStatus, 
            motor_pos: List[float], 
            target_pos: List[float],
            temps: List[float],
            error_status: ArmErrorStatus) -> None:
        """记录一行数据"""
        row = {
            'timestamp': time.time(),
            'state': state.name,
            'error_status': error_status.name
        }
        
        # 电机位置
        for i in range(6):
            row[f'motor_pos_{i}'] = motor_pos[i] if i < len(motor_pos) else 0.0
            row[f'target_pos_{i}'] = target_pos[i] if i < len(target_pos) else 0.0
            row[f'temp_{i}'] = temps[i] if i < len(temps) else 0.0
        
        self._buffer.append(row)
        
        # if len(self._buffer) >= self._buffer_size:
        #     self.flush()
    
    def flush(self) -> None:
        """刷新缓冲区到磁盘"""
        if self._writer and self._buffer:
            self._writer.writerows(self._buffer)
            self._file.flush()
            self._buffer.clear()
    
    def close(self) -> None:
        """关闭日志文件（进程退出时调用）"""
        self.flush()
        if self._file:
            self._file.close()
            print(f"[Device {self.device_id}] CSV日志已保存: {self.file_path}")
