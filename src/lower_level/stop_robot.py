def stop_robot(left_motor, right_motor):
    """
    Stop the robot by setting both motors to zero speed.
    """
    left_motor.set_velocity(0)
    right_motor.set_velocity(0)