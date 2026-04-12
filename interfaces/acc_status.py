from enum import Enum

class AccStatus(Enum):
    OFF = 0
    STANDBY = 1
    CRUISING = 2
    FOLLOWING = 3
    FAULT = 4
