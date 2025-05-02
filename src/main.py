# Library imports
from vex import *
from time import sleep
import time


brain=Brain()


# CONNECT TO PORTS
left_motor = Motor(Ports.PORT1, 18_1, False)
right_motor = Motor(Ports.PORT10, 18_1, True)
camera_motor = Motor(Ports.PORT3, 18_1, False)
claw_motor = Motor(Ports.PORT11, 18_1, True)

ai_vision_2__ORANGE = Colordesc(1, 243, 105, 94, 10, 0.34)
ai_vision_2__YELLOW = Colordesc(2, 231, 163, 92, 12, 0.05)
ai_vision_2__GREEN = Colordesc(3, 24, 202, 84, 10, 0.7)
ai_vision_2 = AiVision(Ports.PORT20, ai_vision_2__ORANGE, ai_vision_2__YELLOW, ai_vision_2__GREEN)

rangeFinderFront = Sonar(brain.three_wire_port.g)
line_sensor_left = Line(brain.three_wire_port.b)
line_sensor_right = Line(brain.three_wire_port.a)

imu = Inertial(Ports.PORT3)
imu.calibrate()
while imu.is_calibrating():
    sleep(.01)


# rangeFinderFront = Sonar(brain.three_wire_port.g)

# TODO: CHANGE PORT
# imu = Inertial(Ports.PORT3)
# imu.calibrate()


claw_motor.set_max_torque(1.5, TorqueUnits.NM)
claw_motor.set_stopping(BRAKE)
claw_motor.reset_position()


camera_motor.set_stopping(HOLD)
camera_motor.set_max_torque(2.0, TorqueUnits.NM)
camera_motor.reset_position()



# CONSTANT VALUES
WHEEL_DIAMETER = 4.0
GEAR_RATIO = 5.0
WHEEL_TRACK = 11.0 
K_P = 4
D_P = 0.8
SET_WALL_FOLLOW_SPEED = 150         # RPM
SET_DISTANCE_TO_START_TURN = 3      # inches from wall in front to begin turn
DISTANCE_TO_SECOND_TURN = 40



def cm_to_degrees(cm):
    return cm * 360 / (WHEEL_DIAMETER * math.pi) * GEAR_RATIO

def turn_degrees(robot_degrees, rpm=100):
    deg = robot_degrees * GEAR_RATIO * WHEEL_TRACK / WHEEL_DIAMETER
    left_motor.spin_for(REVERSE, deg, DEGREES, rpm, RPM, False)
    right_motor.spin_for(FORWARD,   deg, DEGREES, rpm, RPM, True)

# LOWER LEVEL FUNCTIONS
def drive(speed, direction):
   """
   Function to drive the robot with a given speed and direction.
   """
   left_motor.set_velocity(speed, RPM)
   right_motor.set_velocity(speed + direction, RPM)
   left_motor.spin(FORWARD)
   right_motor.spin(FORWARD)

def follow_line():
    "Function to follow a line using two line sensors."

    # difference in reflectivity between the two sensors
    diff = line_sensor_left.reflectivity() - line_sensor_right.reflectivity()
    drive(SET_WALL_FOLLOW_SPEED, -K_P*diff)
    return line_exists()

def follow_line_backwards():
    "Function to follow a line using two line sensors."

    # difference in reflectivity between the two sensors
    diff = line_sensor_left.reflectivity() - line_sensor_right.reflectivity()
    drive(-SET_WALL_FOLLOW_SPEED, K_P*diff)
    return line_exists()

def line_exists(cutoff=8):
    """
    If both sensors detect a line, we should stop the robot because it means it has passed the line intersection.
    """
    r1 = line_sensor_left.reflectivity()
    r2 = line_sensor_right.reflectivity()

    if(r1 < cutoff and r2 < cutoff):
        return False
    return True

def move_robot_forwards(distance):
    DEGREES_PER_CM = 360 / (WHEEL_DIAMETER * 3.14159)  # degrees per cm
    left_motor.spin_for(FORWARD, distance*GEAR_RATIO*DEGREES_PER_CM,DEGREES , 200, RPM, False)
    right_motor.spin_for(FORWARD, distance*GEAR_RATIO*DEGREES_PER_CM, DEGREES, 200, RPM, True)

def stop_robot():
    """
    Stop the robot by setting both motors to zero speed.
    """
    left_motor.set_velocity(0)
    right_motor.set_velocity(0)

def follow_april_tag():
    pass

def face_box_perpendicular():
    pass

def move_forwards_until_touching_box():
    pass

def open_claw():
    claw_move(REVERSE, 0.1)

def move_backwards():
    pass

def raise_camera():
    camera_motor.spin_for(FORWARD, 500)

def lower_camera():
    camera_motor.spin_for(REVERSE, 500)

def turn_still(degrees):
    left_motor.spin_for(REVERSE, degrees*GEAR_RATIO*WHEEL_TRACK/WHEEL_DIAMETER, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, degrees*GEAR_RATIO*WHEEL_TRACK/WHEEL_DIAMETER, DEGREES, 100, RPM, True)

def close_claw():
    claw_move(FORWARD, 0.19)
    
def DetectObject(color):
  objects = ai_vision_2.take_snapshot(color)
  if (objects):
    return [ai_vision_2.largest_object().height, ai_vision_2.largest_object().centerX, ai_vision_2.largest_object().centerY]
  return [None, None, None]

def SetDistanceAway(setWidth, width, setLeft, left):
  to_left = setLeft - left
  to_target = setWidth - width
  velocity = 3
  k_p = 1
  left_motor.set_velocity(to_target * velocity + to_left * k_p)
  right_motor.set_velocity(to_target * velocity - to_left * k_p)

def SetCameraHeight(setHeight, top):
  to_top = setHeight - top
  camera_motor.set_velocity(to_top)

def SetRotation(setLeft, left):
    to_left = setLeft - left
    speed = 0.5
    left_motor.set_velocity(to_left * speed)
    right_motor.set_velocity(-to_left * speed)

def follow_color(color):
    left_motor.spin(FORWARD, False)
    right_motor.spin(FORWARD, False)
    camera_motor.spin(FORWARD, False)
    MAX_DISTANCE = 180
    ERROR = 10
    CAMERA_HEIGHT = 120


    # first, our program move the camera up until it's at the right y height and rotates the robot so it's the right x value
    # then, we have it move forwards until it's close enough to the object

    # set camera height
    while True:
        [height, left, top] = DetectObject(color)

        if(top == None):
            left_motor.set_velocity(0, RPM)
            right_motor.set_velocity(0, RPM)
            camera_motor.set_velocity(0, RPM)

        if(isinstance(top, int) and (CAMERA_HEIGHT - ERROR) < top < (CAMERA_HEIGHT + ERROR)):
            left_motor.set_velocity(0, RPM)
            right_motor.set_velocity(0, RPM)
            camera_motor.set_velocity(0, RPM)
            break

        if(isinstance(top, int)):
            SetCameraHeight(CAMERA_HEIGHT, top)

    # set rotation
    while True:
        [height, left, top] = DetectObject(color)

        if(left == None):
            left_motor.set_velocity(0, RPM)
            right_motor.set_velocity(0, RPM)
            camera_motor.set_velocity(0, RPM)

        if(isinstance(left, int) and left < 304/2 + ERROR and left > 304/2 - ERROR):
            left_motor.set_velocity(0, RPM)
            right_motor.set_velocity(0, RPM)
            camera_motor.set_velocity(0, RPM)
            break

        if(isinstance(left, int)):
            SetRotation(304/2, left)

    # go forwards until the object is close enough to pick up

    max_height = 0

    while True:
        [height, left, top] = DetectObject(color)

        if(height is not None and height > 100):
            if(height > max_height):
                max_height = height

        if(isinstance(height, int) and height > MAX_DISTANCE):
            stop_robot()
            break
        
        if(height != None and isinstance(height, int)):
            SetDistanceAway(MAX_DISTANCE, height, 304/2, left)
        else:
            stop_robot()

        camera_motor.set_velocity(0, RPM)

    move_robot_forwards(2)

def claw_move(direction, threshold=2.0):
    claw_motor.spin(direction)
    current_torque = 0.0

    while True:
        current_torque = claw_motor.torque(TorqueUnits.NM)
        if current_torque >= threshold:
            claw_motor.set_stopping(BRAKE)
            break
        sleep(5/1000)

def drop_off_the_fruit():
    follow_april_tag()
    face_box_perpendicular()
    move_forwards_until_touching_box()
    open_claw()
    move_backwards()

def get_up_ramp():
    """
    Function to get up a ramp using line sensors.
    """
    line_exists = follow_line()
    while line_exists:
        line_exists = follow_line()
    
    move_robot_forwards(24)

    # turn left 90 degrees
    turn_still(-90)

def get_onto_line(right):
    # move forwards a foot to get away from the ramp

    correction = 40
    if right == False:
        correction *= -1

    # until we see a line, keep moving forwards with the right motor slightly faster than the left motor
    while (not line_exists(cutoff=15)) or (line_sensor_left.reflectivity() == 100 or line_sensor_right.reflectivity() == 100):
        left_motor.set_velocity(SET_WALL_FOLLOW_SPEED + correction, RPM)
        right_motor.set_velocity(SET_WALL_FOLLOW_SPEED - correction, RPM)
        left_motor.spin(FORWARD)
        right_motor.spin(FORWARD)

    
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)

    # move robot forward until right sensor is on white and left isn't
    while not (line_sensor_right.reflectivity() > 50 and line_sensor_left.reflectivity() < 12):
        left_motor.set_velocity(SET_WALL_FOLLOW_SPEED)
        right_motor.set_velocity(SET_WALL_FOLLOW_SPEED)

    ERR = 4

    print(line_sensor_left.reflectivity(), line_sensor_right.reflectivity())

    # rotate right until reflection for left and right sensor is within ERR
    while not (-ERR < line_sensor_left.reflectivity() - line_sensor_right.reflectivity() < ERR):
        left_motor.set_velocity(- SET_WALL_FOLLOW_SPEED)
        right_motor.set_velocity(SET_WALL_FOLLOW_SPEED)
        print(line_sensor_left.reflectivity(), line_sensor_right.reflectivity())
    
    stop_robot()


def get_to_box():
    while rangeFinderFront.distance(DistanceUnits.IN) == 0 or rangeFinderFront.distance(DistanceUnits.IN) > 4:
        follow_line()

    left_motor.set_velocity(0, RPM)
    right_motor.set_velocity(0, RPM)

def drop_fruit():
    lower_camera()
    
    open_claw()

    sleep(1)

    move_robot_forwards(-2)
    turn_still(120)

def turn_to_line():
    turn_speed = 20

    # turn to the left until we see a line
    while not line_exists(cutoff=15):
        left_motor.set_velocity(SET_WALL_FOLLOW_SPEED, RPM)
        right_motor.set_velocity(SET_WALL_FOLLOW_SPEED + turn_speed, RPM)
        left_motor.spin(FORWARD)
        right_motor.spin(FORWARD)

    # stop the robot
    stop_robot()

def get_fruit_and_drop_into_box(color, right):
    # raise_camera()
    open_claw()
    follow_color(color)
    close_claw()
    sleep(1) # I don't like this, but for some reason close claw runs asynchronously?
    move_robot_forwards(-15)
    get_onto_line(right)
    # if(right == False):
    #     print("RUNS")
    #     # rotate right until reflection for left and right sensor is within ERR
    #     ERR = 4
    #     left_motor.spin(FORWARD)
    #     right_motor.spin(FORWARD)
    #     while not (-1 * ERR < line_sensor_left.reflectivity() - line_sensor_right.reflectivity() < ERR):
    #         left_motor.set_velocity(-1 * SET_WALL_FOLLOW_SPEED)
    #         right_motor.set_velocity(SET_WALL_FOLLOW_SPEED)
    #         print(line_sensor_left.reflectivity(), line_sensor_right.reflectivity())
    #     stop_robot()
    # else:
    #     get_onto_line(right)
    get_to_box()
    if(right == False):
        turn_still(30)
    
    lower_camera()

    drop_fruit()

    sleep(1)



def dead_reconning_to_beginning():

    drive(100, 0)

    while True:
        if line_exists() and line_sensor_left.reflectivity() != 100 and line_sensor_right.reflectivity() != 100:
            break
        sleep(10/1000)

    stop_robot()
    sleep(100/1000)
    turn_still(180)




# get_up_ramp()
# get_fruit_and_drop_into_box(ai_vision_2__GREEN)

# raise_camera()
# get_to_box()
# drop_fruit()
# move_robot_forwards(12)
# get_fruit_and_drop_into_box(ai_vision_2__ORANGE)

# raise_camera()
# get_to_box()
# drop_fruit()
# get_onto_line()
# sleep(1)


def object_to_right_of_screen(color):
    objects = ai_vision_2.take_snapshot(color)

    object_to_right_of_screen = False

    for n in objects:
        if(n.centerX >= 304/2):
            object_to_right_of_screen = True

    return object_to_right_of_screen


def object_to_left_of_screen(color):
    objects = ai_vision_2.take_snapshot(color)

    object_to_right_of_screen = False

    for n in objects:
        if(n.centerX <= 304/2 and n.height > 50):
            object_to_right_of_screen = True

    return object_to_right_of_screen
    

# # TODO: while doesn't see yellow fruit on right side
# while not object_to_right_of_screen(ai_vision_2__YELLOW):
#     follow_line()

# stop_robot()

# turn_still(30)
# move_robot_forwards(12)
# get_fruit_and_drop_into_box(ai_vision_2__YELLOW)

# left_motor.spin(FORWARD)
# right_motor.spin(FORWARD)



# turn around code

# move_robot_forwards(-3)

# turn_still(90)
# print(line_sensor_left.reflectivity())
# print(line_sensor_right.reflectivity())
# print(line_exists(cutoff=15))

# left_motor.spin(FORWARD)
# right_motor.spin(FORWARD)

# while (not line_exists(cutoff=15)) or (line_sensor_left.reflectivity() == 100 or line_sensor_right.reflectivity() == 100):
#     left_motor.set_velocity(- SET_WALL_FOLLOW_SPEED)
#     right_motor.set_velocity(SET_WALL_FOLLOW_SPEED)

# stop_robot()




get_up_ramp()
raise_camera()
while True:
    get_fruit_and_drop_into_box(ai_vision_2__GREEN, True)
    turn_still(90)
    get_fruit_and_drop_into_box(ai_vision_2__ORANGE, True)
    turn_still(90)
    move_robot_forwards(20)
    get_fruit_and_drop_into_box(ai_vision_2__YELLOW, True)

    turn_still(360)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)

    while (not line_exists(cutoff=15)) or (line_sensor_left.reflectivity() == 100 or line_sensor_right.reflectivity() == 100):
        left_motor.set_velocity(- SET_WALL_FOLLOW_SPEED)
        right_motor.set_velocity(SET_WALL_FOLLOW_SPEED)

    raise_camera()

    while not object_to_left_of_screen(ai_vision_2__ORANGE):
        follow_line()

    turn_still(-30)
    move_robot_forwards(12)

    get_fruit_and_drop_into_box(ai_vision_2__ORANGE, False)