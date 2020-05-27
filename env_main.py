import elevator


def main():
    env = elevator.ElevatorEnv(800, 600)

    while not env.bd.is_done:
        env.step()

    print("Elevator main loop is ended.")


if __name__ == "__main__":
    main()

