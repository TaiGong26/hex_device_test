from enum import Enum

class ArmErrorStatus(Enum):
    Normal           = 0
    Warning          = 1
    ProcessError     = 2
    MotorError       = 3
    ArmError         = 4
    ConnError        = 5
    OtherError       = 6

class ArmCmdStatus(Enum):
    IDLE = 0
    RUN = 1
    BRAKE =2 
    STOPPED =3
    ZERO_STOPPED = 4

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
