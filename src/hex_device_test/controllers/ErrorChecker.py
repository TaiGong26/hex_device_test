from typing import List, Tuple, Optional,Any

from hex_device import Arm, MotorBase, MotorError
from hex_device import public_api_types_pb2

from hex_device_test.statuses.ArmStatus import ArmErrorStatus

class ArmErrorChecker:
    """
    异常检查
    """
    
    @staticmethod
    def check_device(device: Arm) -> Tuple[bool, List[Tuple[ArmErrorStatus,List[Any]]]]:
        """
        全面检查设备状态
        返回: (has_error, error_list)
        list[error_code]
        """
        errors = []
        
        # 2. device error check
        err_dev,err_code_dev,reason = ArmErrorChecker._check_device_error(device)
        if err_dev:
            errors.append((err_code_dev,reason))
        
        # 3. motor error check
        err_motor,err_code_motor,reason = ArmErrorChecker._check_motor_error(device)
        if err_motor:
            reasons = []
            for idx, err in enumerate(reason):
                if err is not None:
                    res = f"motor{idx} err:{MotorError(err).name}"
                    reasons.append(res)
            
            errors.append((err_code_motor,reasons))
        
        return len(errors) > 0, errors
    
    @staticmethod
    def _check_device_error(device:Arm) -> Tuple[bool, ArmErrorStatus, Optional[Any]]:
        """"get device error
        
        但是当前只判断apitimeouts，所以暂时不启用
        
        return bool , reason
        """
        error_info = device.get_parking_stop_detail()
        # if error_info.category == 5:
        #     return True, ArmErrorStatus.ConnError, error_info.category
        
        if error_info.category in (1,2,4,6,7):
            # specific reason in public_api_types.proto enum ParkingStopCategory
            return True, ArmErrorStatus.ArmError, error_info.category
        
        return False, ArmErrorStatus.Normal, None
    
    @staticmethod
    def _check_motor_error(device:Arm) -> Tuple[bool, ArmErrorStatus, Optional[Any]]:
        """"get motor error
        
        return bool , reason
        """
        error_codes = device.get_motor_error_codes()
        if error_codes is None :
            return False, ArmErrorStatus.Normal, None
            
        for i, code in enumerate(error_codes):
            if code is not None:
                return True, ArmErrorStatus.MotorError, error_codes

        return False, ArmErrorStatus.Normal, None
    
    @staticmethod
    def format_error(code: ArmErrorStatus, reason: Any) -> str:
        if code == ArmErrorStatus.MotorError:
            return (f"motor{idx} err:"
                for idx, reason in enumerate(reason)
                if reason is not None)
                
        elif code in (ArmErrorStatus.ConnError, ArmErrorStatus.ArmError):
            return public_api_types_pb2.ParkingStopCategory.Name(reason)