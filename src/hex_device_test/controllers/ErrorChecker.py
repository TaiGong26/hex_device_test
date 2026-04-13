from typing import List, Tuple

from hex_device import Arm

class ArmErrorChecker:
    """
    异常检查接口
    提供暴力扫描device接口进行检查的方法
    """
    
    TEMP_WARNING_THRESHOLD = 70.0   # 温度警告阈值
    TEMP_ERROR_THRESHOLD = 85.0     # 温度错误阈值
    POS_DEVIATION_THRESHOLD = 0.5   # 位置偏差阈值（弧度）
    
    @staticmethod
    def check_device(device: Arm) -> Tuple[bool, List[str]]:
        """
        全面检查设备状态
        返回: (has_error, error_list)
        """
        errors = []
        
        # 1. 检查通信状态
        if not ArmErrorChecker._check_communication(device):
            errors.append("COMM_LOST: 通信中断")
        
        # 2. 检查电机温度
        temp_errors = ArmErrorChecker._check_temperatures(device)
        errors.extend(temp_errors)
        
        # 3. 检查位置偏差
        if not ArmErrorChecker._check_position_deviation(device):
            errors.append("POS_DEVIATION: 位置偏差过大")
        
        # 4. 检查设备特定错误码
        device_errors = ArmErrorChecker._check_device_errors(device)
        errors.extend(device_errors)
        
        return len(errors) > 0, errors
    
    @staticmethod
    def _check_communication(device: Arm) -> bool:
        """检查通信状态"""
        try:
            # 尝试获取位置作为通信测试
            pos = device.get_motor_positions()
            return pos is not None
        except Exception:
            return False
    
    @staticmethod
    def _check_temperatures(device: Arm) -> List[str]:
        """检查电机温度"""
        errors = []
        try:
            temps = device.get_temperatures() if hasattr(device, 'get_temperatures') else []
            for i, temp in enumerate(temps):
                if temp > ArmErrorChecker.TEMP_ERROR_THRESHOLD:
                    errors.append(f"TEMP_ERROR: 电机{i}温度过高 ({temp:.1f}°C)")
                elif temp > ArmErrorChecker.TEMP_WARNING_THRESHOLD:
                    errors.append(f"TEMP_WARNING: 电机{i}温度警告 ({temp:.1f}°C)")
        except Exception as e:
            errors.append(f"TEMP_CHECK_FAILED: 温度检查失败 ({e})")
        return errors
    
    @staticmethod
    def _check_position_deviation(device: Arm) -> bool:
        """检查实际位置与目标位置偏差"""
        try:
            # 这里需要根据实际情况获取目标位置
            # 简化处理：仅检查位置有效性
            pos = device.get_motor_positions()
            return pos is not None and all(abs(p) < 10 for p in pos)  # 合理性检查
        except Exception:
            return False
    
    @staticmethod
    def _check_device_errors(device: Arm) -> List[str]:
        """检查设备特定的错误码"""
        errors = []
        # 预留接口，根据实际device接口实现
        # if hasattr(device, 'get_error_code'):
        #     error_code = device.get_error_code()
        #     if error_code != 0:
        #         errors.append(f"DEVICE_ERROR: 设备错误码 {error_code}")
        return errors