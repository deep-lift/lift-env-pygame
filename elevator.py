import sys
from enum import Enum
import pygame as pg
from argslist import *
import numpy as np


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
    elevator_count: int = N_AGENTS
    floors: int = 10
    passenger: int = N_PASSENGER
    height: float = 3.5
    speed: float = 3
    decelerate: float = 1
    accelerate: float = 1
    open: float = 1
    close: float = 1
    turn: float = 1
    capacity: int = 15
    action_to_floor: int = 1
    fixedTime: float = FIXED_TIME
    max_step: int = N_MAX_STEPS
    bd: building
    heuristic: bool = True
    fixed_scenario:bool = True   #고정된 시나리오 동작 여부 플래그
    
    observations = np.zeros((N_AGENTS, N_OBSERVATION))
    states = np.zeros(N_STATE)
    rewards = np.asarray((N_AGENTS,))
    dones = np.asarray((N_AGENTS,))

    def __init__(self, size_x: int, size_y: int, heuristic: bool = True):
        pg.init()
        self.screen = pg.display.set_mode([size_x, size_y], pg.DOUBLEBUF)
        self.display = pg.display.set_caption("Deep Lift")
        self.clock = pg.time.Clock()
        self.bd = building.Building(self, self.screen)
        self.heuristic = heuristic
        self.total_reward = 0
        self.display = True

    def step(self, actions: list):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()
            if event.type == pg.KEYDOWN:
                keys = pg.key.get_pressed()
                if keys[pg.K_F12]:
                    self.display = not self.display
                  

        if RENDER:
            self.clock.tick(GAME_SPEED)

        if not self.heuristic: 
            self.bd.decision_actions(actions)

        observations, rewards, dones, requested_agents = self.bd.update_step()

        # requested_agents = [False] * N_AGENTS

        # 재영님 여기서 첫번째꺼 31개 짜르면 나머지 엘베도 다 공통이죠?
        self.states = observations[0][0:31]

        for a in range(N_AGENTS):
            self.observations[a] = observations[45*a+31:45*a+45]

        if self.display:
            self.render()

        return observations, rewards, dones, requested_agents

    def render(self):
        self.bd.render()
        pg.display.update()

    def step_split(self, actions:list):
        observations, rewards, dones, requested_agents = self.step(actions)
        return rewards, dones, requested_agents

    def get_obs(self):
        return self.observations

    def get_state(self):
        return self.states

    def reset(self):
        self.bd.reset()
        return self.bd.env_info()


if __name__ == '__main__':
    print(ElevatorEnv.fixedTime)
    print(MoveState.STOP)