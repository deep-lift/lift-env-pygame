import elevator
from argslist import *

def main():
    env = elevator.ElevatorEnv(SCREEN_HEIGHT, SCREEN_WIDTH)

    while not env.bd.is_done:
        env.step()
    print("Elevator main loop is ended.")


if __name__ == "__main__":
    main()

