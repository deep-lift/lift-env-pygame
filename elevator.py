import sys
from enum import Enum
import pygame as pg
from argslist import *


class MoveState(Enum):
    STOP = 0
    DOWN = 1
    UP = 2

class Passenger:
    def __init__(self, start: int, dest: int):
        self.start_floor = start
        self.dest_floor = dest
        self.wait_time = 0


import building
from render import *



class ElevatorEnv:
    elevator_count: int = 4
    floors: int = 10
    passenger: int = 100
    height: float = 3.5
    speed: float = 3
    decelerate: float = 1
    acelerate: float = 1
    open: float = 1
    close: float = 1
    turn: float = 1
    capacity: int = 15
    actionTofloor: int = 0
    fixedTime: float = FIXED_TIME
    maxstep: int = 5000
    bd:building
    heuristic: bool = True

    def __init__(self, size_x: int, size_y: int, heuristic:bool = True):
        pg.init()
        self.screen = pg.display.set_mode([size_x, size_y], pg.DOUBLEBUF)
        self.display = pg.display.set_caption("elevator")
        self.clock = pg.time.Clock()
        self.bd = building.Building(self, self.screen)
        self.heuristic = heuristic

    def step(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()

        self.clock.tick(100)
        self.bd.update_step()
        self.bd.render()

        pg.display.update()


    def step(self,actions:list):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()
    
        self.clock.tick(100)

        if not self.heuristic: 
            self.bd.decision_actions(actions)

        states,rewards,dones = self.bd.update_step()
        self.bd.render()
        pg.display.update()
        return states,rewards,dones
      
       
    def reset(self):
        self.bd.reset()

        return  self.bd.env_info()








if __name__ == '__main__':
    print(ElevatorEnv.fixedTime)
    print(MoveState.STOP)