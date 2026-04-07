from dataclasses import dataclass, field
from typing import List, Optional
import copy


"""
{
    'name':'Archer_d6y',
    'dof_num': 'six_axis',
    'motor_model': [0x80] * 6,
    'joints': [{
        'joint_name': 'joint_1',
        'joint_limit': [-2.7, 2.7, -0.1, 0.1, 0.0, 0.0]
    }, {
        'joint_name': 'joint_2',
        'joint_limit': [-1.57, 2.094, -SPEED, SPEED, 0.0, 0.0]
    }, {
        'joint_name': 'joint_3',
        'joint_limit': [0.0, 3.14159265359, -SPEED, SPEED, 0.0, 0.0]
    }, {
        'joint_name': 'joint_4',
        'joint_limit': [-1.5, 1.5, -SPEED, SPEED, 0.0, 0.0]
    }, {
        'joint_name': 'joint_5',
        'joint_limit': [-1.56, 1.56, -SPEED, SPEED, 0.0, 0.0]
    }, {
        'joint_name': 'joint_6',
        'joint_limit': [-1.57, 1.57, -SPEED, SPEED, 0.0, 0.0]
    }]
}

"""

@dataclass(frozen=True)
class JointConfig:
    joint_name: str
    joint_limit: List[float]

@dataclass(frozen=True)
class ArmConfig:

    name: str
    dof_num: str
    motor_model: List[int]
    joints: List[JointConfig]

    @classmethod
    def read_config_from_dict(cls, config_dict: dict):
        return cls(
            name = config_dict['name'],
            dof_num = config_dict['dof_num'],
            motor_model = config_dict['motor_model'],
            joints = [JointConfig(**joint) for joint in config_dict['joints']]
        )
    
    def config_to_dict(self):
        return self.__dict__