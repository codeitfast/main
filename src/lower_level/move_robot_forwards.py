from vex import *

def move_robot_forwards(left_motor, right_motor, distance, WHEEL_DIAMETER, GEAR_RATIO):
    # Convert distance in feet to inches
    distance_in_inches = distance * 12
    # Calculate the number of rotations needed to cover the distance
    rotations_needed = distance_in_inches / (WHEEL_DIAMETER * 3.14159)
    # Adjust rotations needed by the gear ratio
    rotations_needed /= GEAR_RATIO
    # Calculate the time in milliseconds to cover the distance at max speed (100% RPM)
    max_speed_rpm = 200  # Assuming 200 RPM is the max speed of the motor
    time_needed = (rotations_needed / (max_speed_rpm / 60)) * 1000

    # Set motors to max speed and drive
    left_motor.set_velocity(max_speed_rpm, RPM)
    right_motor.set_velocity(max_speed_rpm, RPM)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)