# from elevator import MoveState
from enum import Enum
from elevator import *

# class MoveState(Enum):
#     stop = 0
#     down = 1
#     up = 2


def check(floor:int, direction: MoveState):
    print('floor: ' + str(floor))
    print('direction : {}'.format(direction))


check(3, MoveState.DOWN)