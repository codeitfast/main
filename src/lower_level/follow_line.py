from drive import drive

def follow_line(left_motor, right_motor, lineSensorLeft, lineSensorRight, SET_WALL_FOLLOW_SPEED, K_P):
    "Function to follow a line using two line sensors."
    lineSensorLeft.reflectivity()
    lineSensorRight.reflectivity()

    # difference in reflectivity between the two sensors
    diff = lineSensorLeft.reflectivity() - lineSensorRight.reflectivity()

    # reset velocities
    drive(left_motor, right_motor, SET_WALL_FOLLOW_SPEED, K_P*0)

    # correct to line
    if (diff) < -3 or (diff) > 3:
        drive(left_motor, right_motor, SET_WALL_FOLLOW_SPEED, K_P*diff)