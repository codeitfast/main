from vex import *

def drive(left_motor, right_motor, speed, direction):
   """
   Function to drive the robot with a given speed and direction.
   """
   left_motor.set_velocity(speed, RPM)
   right_motor.set_velocity(speed + direction, RPM)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)