from picarx_improved import Picarx

px = Picarx()
running = True

while running == True:
    value = input("Enter maneuver type:")

    if value == "a":  #forward and backward
        speed = input("Enter speed value:")
        speed = int(speed)
        angle = input("Enter angle value (degrees):")
        speed = int(speed)
        time = input("Enter time of travel (seconds):")
        time = int(time)

        px.forward_and_backward(speed, angle, time)

    if value == "b": #parallel parking
        direction = input("Enter side to park (left or right):")

        px.parallel_park(direction)

    if value == "c": #k turn
        direction = input("Enter side to turn (left or right):")

        px.k_turn(direction)

    if value == "d": #exit
        running = False
        break

