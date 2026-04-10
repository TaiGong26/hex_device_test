from enum import Enum

class ArmErrorStatus(Enum):
    Normal      = 1
    Warning     = 2
    Error       = 3

# class 

class ArmCoordinatorStatus(Enum):
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    Stopped = 4
    Error = 5
    Exit = 6

class ArmControllerStatus(Enum):
    Disconnected = 0
    Init = 1
    Ready = 2
    Running = 3
    Brake = 4
    Stopped = 5
    Exit = 6

