import elevator
from argslist import *
import random

def main():
    env = elevator.ElevatorEnv(SCREEN_WIDTH,SCREEN_HEIGHT,False)

    stats,rewards,dones = env.reset()

    
    while not env.bd.is_done:
        actions = []
        for  reward in rewards:
            actions.append(random.randint(0,2))

        states,rewards,dones = env.step(actions)

    print("Elevator main loop is ended.")


if __name__ == "__main__":
    main()

