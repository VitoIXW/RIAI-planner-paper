from enum import Enum

class AssignationMethods(Enum):
    RRT = 0
    RRT_STAR = 1
    RRT_STAR_HUNGARIAN = 2
    RANDOM = 3

class RRTType(Enum):
    RRT = 0
    RRT_STAR = 1