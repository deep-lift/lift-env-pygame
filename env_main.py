import elevator
from argslist import *
import random
import numpy as np


def select_actions(observations, requested_agents):
    actions = []
    n_actions = 0

    # todo : 에이전트 붙이는 부분! observations > actions
    for t in requested_agents:
        if t:
            actions.append(random.randint(0, 2))
            n_actions = n_actions + 1
    print(n_actions)

    return actions


def main():
    env = elevator.ElevatorEnv(SCREEN_WIDTH, SCREEN_HEIGHT, False)
    observations, _, _ = env.reset()
    done = False
    step = 0
    observation = None

    while not done:
        requested_agents = [False] * N_AGENTS
        step = step + 1

        for i, o in enumerate(observations):
            if len(o) != 0:
                requested_agents[i] = True
                if observation is None:
                    observation = o

        if any(requested_agents):
            observations = np.asarray(observation).reshape(N_AGENTS, (N_OBSERVATION+N_STATE))
            actions = select_actions(observations, requested_agents)
            observations, rewards, dones,_ = env.step(actions)
            done = all(dones)
        else:
            observations, rewards, dones,_ = env.step([])
            print('{} step : no request from agents'.format(step))

    print("Deep Lift Game is Ended..")


if __name__ == "__main__":
    main()
