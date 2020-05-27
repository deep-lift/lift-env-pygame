from enum import Enum


class MoveState(Enum):
    stop = 0
    down = 1
    up = 2


class Passenger:
    def __init__(self, start: int, dest: int):
        self.start_floor = start
        self.dest_floor = dest
        self.wait_time = 0

FIXED_TIME = 0.1


import building
import pygame as pg
import sys


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

    def __init__(self, size_x: int, size_y: int):
        pg.init()
        self.screen = pg.display.set_mode([size_x, size_y], pg.DOUBLEBUF)
        self.display = pg.display.set_caption("elevator")
        self.clock = pg.time.Clock()
        self.init_env()

    def init_env(self):
        self.bd = building.Building(self, self.screen)

    def step(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()

        self.clock.tick(1)
        self.bd.update_step()
        self.bd.render()

        pg.display.update()


if __name__ == '__main__':
    print(ElevatorEnv.fixedTime)
    print(MoveState.stop)