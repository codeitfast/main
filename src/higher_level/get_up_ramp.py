from lower_level.follow_line import follow_line
from lower_level.stop_robot import stop_robot
from lower_level.move_robot_forwards import move_robot_forwards

def get_up_ramp(left_motor, right_motor, lineSensorLeft, lineSensorRight, SET_WALL_FOLLOW_SPEED, K_P, WHEEL_DIAMETER, GEAR_RATIO):
    """
    Function to get up a ramp using line sensors.
    """
    line_exists = follow_line(left_motor, right_motor, lineSensorLeft, lineSensorRight, SET_WALL_FOLLOW_SPEED, K_P)
    while line_exists:
        line_exists = follow_line(left_motor, right_motor, lineSensorLeft, lineSensorRight, SET_WALL_FOLLOW_SPEED, K_P)

    stop_robot(left_motor, right_motor)
    
    move_robot_forwards(left_motor, right_motor, 12, WHEEL_DIAMETER, GEAR_RATIO)