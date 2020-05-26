import elevator


#def main():
#    el = elevator.elevator_env();
#
#if __name__ == "__main__":
#    main()

env = elevator.elevator_env(800,600);

while env.bd.isDone == False:
    env.step()


print("End!!!!!!!!!");