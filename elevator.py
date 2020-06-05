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
    elevator_count: int = 4
    floors: int = 10
    passenger: int = 100
    height: float = 3.5
    speed: float = 3
    decelerate: float = 1
    accelerate: float = 1
    open: float = 1
    close: float = 1
    turn: float = 1
    capacity: int = 15
    action_to_floor: int = 0
    fixedTime: float = FIXED_TIME
    max_step: int = 50000
    bd: building
    heuristic: bool = True

    observations = np.zeros((N_AGENTS, N_OBSERVATION))
    states = np.zeros((N_AGENTS, N_STATE))
    rewards = np.asarray((N_AGENTS,))
    dones = np.asarray((N_AGENTS,))

    def __init__(self, size_x: int, size_y: int, heuristic: bool = True):
        pg.init()
        self.screen = pg.display.set_mode([size_x, size_y], pg.DOUBLEBUF)
        self.display = pg.display.set_caption("Deep Lift")
        self.clock = pg.time.Clock()
        self.bd = building.Building(self, self.screen)
        self.heuristic = heuristic

    def step(self, actions: list):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                sys.exit()

        self.clock.tick(GAME_SPEED)

        if not self.heuristic: 
            self.bd.decision_actions(actions)

        observations, rewards, dones = self.bd.update_step()

        requested_agents = [False] * N_AGENTS
        all_observation = None

        for i, o in enumerate(observations):
            if len(o) != 0:
                requested_agents[i] = True
                if all_observation is None:
                    all_observation = o

        # todo : observations에서 observations(쉐어정보), states(개별정보)로 분리하여 저장
        if all_observation is not None:
            self.states = all_observation[0:31]

            for a in range(N_AGENTS):
                self.observations[a] = all_observation[46*a+31:46*a+46]

        if RENDER:
            self.render()

        return observations, rewards, dones

    def render(self):
        self.bd.render()
        pg.display.update()

    def step_split(self, actions:list):
        observations, rewards, dones = self.step(actions)
        return rewards, dones, []

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