#region VEXcode Generated Robot Configuration
from vex import *
'''from DRIVER_FUNCTIONS.drive import ondriver_drivercontrol_0, ondriver_drivercontrol_1, ondriver_drivercontrol_2, ondriver_drivercontrol_3, onauton_autonomous_0, onevent_controller_1axis2Changed_0, onevent_controller_1axis3Changed_0
from AUTO.autoselect import onevent_controller_1buttonL1_pressed_0
from INIT.init import when_started4, onevent_stop_initialize_0
from DRIVER_FUNCTIONS.LB import ondriver_drivercontrol_4
from AUTO.autonomous import onauton_autonomous_0
from OTHER.testcode import when_started2, when_started3
from AUTO.autoselect import when_started5'''
import urandom # type: ignore

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
RightMotors_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
RightMotors_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
RightMotors = MotorGroup(RightMotors_motor_a, RightMotors_motor_b)
LeftMotors_motor_a = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
LeftMotors_motor_b = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
LeftMotors = MotorGroup(LeftMotors_motor_a, LeftMotors_motor_b)
Right_front = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
Left_Front = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
digital_out_b = DigitalOut(brain.three_wire_port.b)
rotation_15 = Rotation(Ports.PORT15, False)
Lady_Brown_motor_a = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
Lady_Brown_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
Lady_Brown = MotorGroup(Lady_Brown_motor_a, Lady_Brown_motor_b)
Inertial21 = Inertial(Ports.PORT5)
digital_out_g = DigitalOut(brain.three_wire_port.g)
optical_4 = Optical(Ports.PORT4)
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
digital_out_e = DigitalOut(brain.three_wire_port.e)
gps_3 = Gps(Ports.PORT3, 0.00, 5.00, MM, -90)
distance_11 = Distance(Ports.PORT11)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


"""# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()"""


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

remote_control_code_enabled = True
vexcode_brain_precision = 0
vexcode_console_precision = 0
vexcode_controller_1_precision = 0
message1 = Event()
forward_move = Event()
Back_move = Event()
Stop = Event()
turn_right = Event()
'''turn = Event()'''
calibrate = Event()
stop_initialize = Event()
Auto_Stop = Event()
turn_left = Event()
start_auto = Event()
intake_forward = Event()
intake_backward = Event()
DOon = False
LB = False
DOon2 = False
Blue = False
Red = False
Intake_Control = False
Intake_running = False
myVariable = 0
volocity = 0
Right_Axis = 0
Left_Axis = 0
IntakeStake = 0
Degree = 0
pi = 0
movement = 0
distance1 = 0
time1 = 0
rot = 0
turn1 = 0
LadyBrown_Up = 0
LadyBrown_score = 0
LadyBrown = 0
Right_turn = 0
Left_turn = 0
DriveState = 0
start = 0
Next = 0
dos = 0
tog = 0
error = 0
output = 0
Kp = 0
Ki = 0
Kd = 0
Dellay = 0
Distance_travled = 0
imput = 0
Proportional = 0
integral = 0
derivitive = 0
direction = 0
Previus_error = 0
AutoSelect = 0
X_Start = 0
Y_Start = 0
Y_End = 0
X_End = 0
Angle = 0
Distnce2 = 0
Distance2 = 0
Turn_Angle = 0



# Clear the screen and set pen color
brain.screen.clear_screen()
brain.screen.set_pen_color(Color.WHITE)

# Draw "169" using rectangles & circles
brain.screen.draw_rectangle(20, 50, 20, 50)  # "1" (Vertical line)
brain.screen.draw_circle(80, 75, 25)         # "6" (Circle)
brain.screen.draw_rectangle(105, 50, 20, 50)  # "9" (Straight part)
brain.screen.draw_circle(115, 75, 25)         # "9" (Circle)

# Draw "B" using rectangles & arcs
brain.screen.draw_rectangle(160, 50, 20, 50)  # Straight part of "B"
brain.screen.draw_circle(170, 65, 15)         # Top curve of "B"
brain.screen.draw_circle(170, 85, 15)         # Bottom curve of "B"

# Keep display on
while True:
    wait(1, SECONDS)
def onevent_controller_1axis2Changed_0():
    global Right_Axis, dead_zone_range
    dead_zone_range = 10  # Adjust this value to set the dead zone range

    Right_Axis = controller_1.axis2.position()

    # Check if joystick is within dead zone
    if abs(Right_Axis) <= dead_zone_range:
        Right_Axis = 0  # Assign a value of 0 when joystick is within dead zone
    else:
        # Otherwise, use the joystick value
        Right_Axis = Right_Axis

def ondriver_drivercontrol_0():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # INTAKE CONTROLLER CONTROL
    while True:
        while Intake_Control:
            if controller_1.buttonR1.pressing():
                intake.set_velocity(80, PERCENT)
                intake.spin(FORWARD)
            elif controller_1.buttonR2.pressing():
                intake.set_velocity(80, PERCENT)
                intake.spin(REVERSE)
            else:
                intake.stop()
            wait(5, MSEC)
        wait(5, MSEC)

'''def onevent_controller_1axis3Changed_0():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER jOYSTICK
    Left_Axis = controller_1.axis3.position()'''

def onevent_controller_1axis3Changed_0():
    global Left_Axis, dead_zone_range
    dead_zone_range = 10  # Adjust this value to set the dead zone range

    Left_Axis = controller_1.axis3.position()

    # Check if joystick is within dead zone
    if abs(Left_Axis) <= dead_zone_range:
        Left_Axis = 0  # Assign a value of 0 when joystick is within dead zone
    else:
        # Otherwise, use the joystick value
        Left_Axis = Left_Axis


'''def ondriver_drivercontrol_1():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER MOTOR VELOCITY CONTROL
    remote_control_code_enabled = True
    DriveState = 1
    volocity = 200
    RightMotors.set_stopping(COAST)
    LeftMotors.set_stopping(COAST)
    Right_front.set_stopping(COAST)
    Left_Front.set_stopping(COAST)
    while True:
        if Right_Axis > 0:
            Right_Axis =  21.6 * Right_Axis ** 1/3
        else:
            Right_Axis = 21.6 * -(math.fabs(Right_Axis) ** 1/3)

        if Left_Axis > 0:
            Left_Axis = 21.6 * (Left_Axis) ** 1/3
        else:
            Left_Axis = 21.6 * -(math.fabs(Left_Axis) ** 1/3)

        while True:
            RightMotors.set_velocity(Right_Axis, PERCENT)
            LeftMotors.set_velocity(Left_Axis, PERCENT)
            Right_front.set_velocity(Right_Axis, PERCENT)
            Left_Front.set_velocity(Left_Axis, PERCENT)
            RightMotors.spin(FORWARD)
            LeftMotors.spin(REVERSE)
            Right_front.spin(FORWARD)
            Left_Front.spin(REVERSE)
            wait(5, MSEC)
        wait(5, MSEC)'''

import math

# Function to limit how fast the output can change (slew rate limiting)
def slew_rate_limit(current, previous, max_delta=5):
    delta = current - previous
    if abs(delta) > max_delta:
        return previous + max_delta * (1 if delta > 0 else -1)
    return current

def ondriver_drivercontrol_1():
    # Set drive motors to coast for smoother motion
    LeftMotors.set_stopping(COAST)
    Left_Front.set_stopping(COAST)
    RightMotors.set_stopping(COAST)
    Right_front.set_stopping(COAST)

    max_velocity = 100  # Maximum motor speed (percent)

    # Initialize previous outputs for slew rate limiting
    previous_left_output = 0
    previous_right_output = 0

    while True:
        # Read joystick values (assumed to be from -100 to 100)
        left_input = Left_Axis
        right_input = Right_Axis


        # Normalize the inputs to the range -1.0 to 1.0
        left_normalized = left_input / 100.0
        right_normalized = right_input / 100.0

        # Apply cubic scaling for smoother, less sensitive control at lower speeds
        # This gives a fine response near zero and full power at the extremes.
        left_cubic = left_normalized ** 3
        right_cubic = right_normalized ** 3

        # Scale the cubic outputs by the maximum velocity
        desired_left_output = left_cubic * max_velocity
        desired_right_output = right_cubic * max_velocity

        # Apply slew rate limiting to smooth out rapid changes in command
        left_output = slew_rate_limit(desired_left_output, previous_left_output, max_delta=5)
        right_output = slew_rate_limit(desired_right_output, previous_right_output, max_delta=5)

        # Save current outputs for the next iteration
        previous_left_output = left_output
        previous_right_output = right_output

        # Set the motor velocities based on the computed outputs
        LeftMotors.set_velocity(left_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Right_front.set_velocity(right_output, PERCENT)

        # Spin the motors (adjust spin directions as needed for your drivetrain)
        LeftMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        RightMotors.spin(FORWARD)
        Right_front.spin(FORWARD)

        wait(20, MSEC)


def ondriver_drivercontrol_2():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER CLAMP CONTROL
    while True:
        if controller_1.buttonB.pressing():
            if DOon:
                digital_out_b.set(True)
                DOon = False
                wait(0.1, SECONDS)
            else:
                digital_out_b.set(False)
                DOon = True
                wait(0.1, SECONDS)
            wait(0.5, SECONDS)
        wait(5, MSEC)

def ondriver_drivercontrol_3():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER SWEEPER CONTROL
    while True:
        if controller_1.buttonA.pressing():
            if DOon2:
                digital_out_g.set(True)
                DOon2 = False
                wait(0.1, SECONDS)
            else:
                digital_out_g.set(False)
                DOon2 = True
                wait(0.1, SECONDS)
            wait(0.5, SECONDS)
        wait(5, MSEC)



def ondriver_drivercontrol_4():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # LADY BROWN MACROS AND INIT
    Lady_Brown.spin_to_position(0, DEGREES)
    rotation_15.set_position(300, DEGREES)
    Lady_Brown.set_timeout(1, SECONDS)
    Lady_Brown.set_velocity(40, PERCENT)
    while True:
        if controller_1.buttonDown.pressing() and 198 < rotation_15.position(DEGREES):
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)
            while not rotation_15.position(DEGREES) < 276:
                wait(5, MSEC)
            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()
        if controller_1.buttonUp.pressing():
            Lady_Brown.set_velocity(100, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)
            while not rotation_15.position(DEGREES) < 152:
                wait(5, MSEC)
            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()
        if controller_1.buttonLeft.pressing():
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(REVERSE)
            while not 300 < rotation_15.position(DEGREES):
                wait(5, MSEC)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.stop()
        wait(5, MSEC)

def when_started4():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CALIBRATE AND INIT
    optical_4.gesture_disable()
    optical_4.set_light(LedStateType.ON)
    optical_4.set_light_power(50, PERCENT)
    start = 1
    Degree = 0
    pi = 3.14159265359
    Lady_Brown.set_velocity(100, PERCENT)
    Lady_Brown.set_position(0, DEGREES)
    intake.set_velocity(80, PERCENT)
    movement = 0
    Intake_Control = True

def onevent_stop_initialize_0():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # INIT
    RightMotors.set_stopping(BRAKE)
    LeftMotors.set_stopping(BRAKE)
    Right_front.set_stopping(BRAKE)
    Left_Front.set_stopping(BRAKE)
    Lady_Brown.set_stopping(BRAKE)
    intake.set_stopping(BRAKE)
    RightMotors.set_velocity(0, PERCENT)
    LeftMotors.set_velocity(0, PERCENT)
    Right_front.set_velocity(0, PERCENT)
    Left_Front.set_velocity(0, PERCENT)
    RightMotors.set_max_torque(100, PERCENT)
    LeftMotors.set_max_torque(100, PERCENT)
    Right_front.set_max_torque(100, PERCENT)
    Left_Front.set_max_torque(100, PERCENT)

def when_started2():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    while True:
        if Red:
            while Red:
                if optical_4.color() == Color.BLUE and optical_4.is_near_object():
                    digital_out_e.set(True)
                    while not (optical_4.color() == Color.RED and optical_4.is_near_object()):
                        wait(5, MSEC)
                    digital_out_e.set(False)
                wait(5, MSEC)
        elif Blue:
            while Blue:
                if optical_4.color() == Color.RED and optical_4.is_near_object():
                    digital_out_e.set(True)
                    while not (optical_4.color() == Color.BLUE and optical_4.is_near_object()):
                        wait(5, MSEC)
                    digital_out_e.set(False)
                wait(5, MSEC)
        else:
            controller_1.screen.print("NO AUTO!!!")
        wait(5, MSEC)


def when_started3():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # SENSING FOR ANTI-OBSTRUTION - GLOBAL
    while True:
        while Intake_running:
            if not intake.is_spinning():
                intake.spin(REVERSE)
                wait(0.5, SECONDS)
            wait(5, MSEC)
        wait(5, MSEC)


def Move_In_direction_Degree_Speed(Move_In_direction_Degree_Speed__Degree, Move_In_direction_Degree_Speed__Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # LINKED WITH CURRENT PID FOR MOVEMENT VELOCITY
    if Move_In_direction_Degree_Speed__Degree > 0:
        RightMotors.set_velocity((Move_In_direction_Degree_Speed__Speed - Move_In_direction_Degree_Speed__Degree), PERCENT)
        Right_front.set_velocity((Move_In_direction_Degree_Speed__Speed - Move_In_direction_Degree_Speed__Degree), PERCENT)
        LeftMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Left_Front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
    else:
        RightMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Right_front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        LeftMotors.set_velocity((Move_In_direction_Degree_Speed__Speed + Move_In_direction_Degree_Speed__Degree), PERCENT)
        Left_Front.set_velocity((Move_In_direction_Degree_Speed__Speed + Move_In_direction_Degree_Speed__Degree), PERCENT)


def Forward_PID_Distance_Max_Speed(Forward_PID_Distance_Max_Speed__Distance, Forward_PID_Distance_Max_Speed__Max_Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CURRENT PID - DISTANCE IN INCHES
    # PID VALUES: 0.5 - 0.01 - 0.1 - 0.05
    error = 0
    Kp = 0.45
    Ki = 0
    Kd = 0.01
    Dellay = 0.05

    Distance_travled = 0
    Inertial21.set_rotation(0, DEGREES)
    RightMotors.set_velocity(5, PERCENT)
    Right_front.set_velocity(5, PERCENT)
    LeftMotors.set_velocity(5, PERCENT)
    Left_Front.set_velocity(5, PERCENT)
    RightMotors.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)
    LeftMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)
    while True:
        imput = Inertial21.rotation(DEGREES)
        error = 0 - imput
        Proportional = error
        Distance_travled = ((math.fabs(Right_front.position(DEGREES)) / 360) * (2.75 * pi) + ((math.fabs(Left_Front.position(DEGREES)) / 360) * (2.75 * pi) + ((math.fabs(RightMotors.position(DEGREES)) / 360) * (2.75 * pi) + (math.fabs(LeftMotors.position(DEGREES)) / 360) * (2.75 * pi)))) / 4
        integral = (integral + error) * Dellay
        derivitive = (error - Previus_error) * Dellay
        direction = Kp * Proportional + (Ki * integral + Kd * derivitive)
        Previus_error = error
        Move_In_direction_Degree_Speed((direction * Forward_PID_Distance_Max_Speed__Max_Speed) / 33, Forward_PID_Distance_Max_Speed__Max_Speed)
        RightMotors.spin(REVERSE)
        Right_front.spin(REVERSE)
        LeftMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        if Forward_PID_Distance_Max_Speed__Distance < Distance_travled:
            RightMotors.stop()
            Right_front.stop()
            LeftMotors.stop()
            Left_Front.stop()
            break
        wait(5, MSEC)





        


# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    
    auton_task_0 = Thread( onauton_autonomous_0 )
    # wait for the driver control period to end
    while( competition.is_autonomous() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the autonomous control tasks
    auton_task_0.stop()

def vexcode_driver_function():
    # Start the driver control tasks
    driver_control_task_0 = Thread( ondriver_drivercontrol_0 )
    driver_control_task_1 = Thread( ondriver_drivercontrol_1 )
    driver_control_task_2 = Thread( ondriver_drivercontrol_2 )
    driver_control_task_3 = Thread( ondriver_drivercontrol_3 )
    driver_control_task_4 = Thread( ondriver_drivercontrol_4 )

    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the driver control tasks
    driver_control_task_0.stop()
    driver_control_task_1.stop()
    driver_control_task_2.stop()
    driver_control_task_3.stop()
    driver_control_task_4.stop()


# register the competition functions
competition = Competition( vexcode_driver_function, vexcode_auton_function )

# system event handlers
controller_1.axis2.changed(onevent_controller_1axis2Changed_0)
controller_1.axis3.changed(onevent_controller_1axis3Changed_0)
stop_initialize(onevent_stop_initialize_0)
controller_1.buttonL1.pressed(onevent_controller_1buttonL1_pressed_0)
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

ws2 = Thread( when_started2 )
ws3 = Thread( when_started3 )
ws4 = Thread( when_started4 )
ws5 = Thread( when_started5 )

import math

def angle_diff(target, current):
    """
    Computes the shortest (signed) difference between two angles (in degrees).
    Returns a value between -180 and 180.
    """
    diff = (target - current + 180) % 360 - 180
    return diff

def pid_drive(distance_inches, max_velocity, timeout=5.0):
    """
    Drives the robot forward for a given distance in inches.
    Uses a PID loop with encoder feedback for distance control and
    an IMU-based proportional correction to maintain heading.
    """
    # ----- PID Constants for Distance Control -----
    kP_distance = 0.5    # Proportional gain for distance error
    kI_distance = 0.0    # Integral gain (can be tuned if needed)
    kD_distance = 0.1    # Derivative gain for distance error

    # ----- PID Constant for Heading Correction -----
    kP_angle = 0.1       # Proportional gain for heading error

    dt = 0.02  # Loop time (20 ms)
    tolerance_distance = 0.5  # Acceptable distance error in inches
        # Maximum motor speed (in percent)

    # ----- Reset Encoders and Set Initial Heading -----
    RightMotors.set_velocity(5, PERCENT)
    Right_front.set_velocity(5, PERCENT)
    LeftMotors.set_velocity(5, PERCENT)
    Left_Front.set_velocity(5, PERCENT)
    RightMotors.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)
    LeftMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)

    # Save the initial heading from the IMU as the target heading
    target_heading = Inertial21.rotation()  # desired heading for straight travel

    integral_distance = 0.0
    previous_error_distance = 0.0
    start_time = brain.timer.time(SECONDS)

    while True:
        # ----- Calculate Traveled Distance -----
        # Read the left and right motor encoder values (in degrees)
        left_deg = LeftMotors.position(DEGREES)+Left_Front.position(DEGREES)
        right_deg = RightMotors.position(DEGREES)+ Right_front.position(DEGREES)
        avg_deg = (left_deg + right_deg) / 4.0

        # Convert encoder degrees to inches.
        # Adjust wheel_diameter if your wheels are not 4 inches.
        wheel_diameter = 2.75  # inches  
        wheel_circumference = wheel_diameter * math.pi  # inches per revolution
        traveled_inches = (avg_deg / 360.0) * wheel_circumference

        # ----- Distance Error Calculation -----
        error_distance = distance_inches - traveled_inches

        # Break out if we've reached our target (or timed out)
        if abs(error_distance) < tolerance_distance or brain.timer.time(SECONDS) - start_time > timeout:
            break

        # ----- PID Calculation for Distance -----
        integral_distance += error_distance * dt
        derivative_distance = (error_distance - previous_error_distance) / dt
        previous_error_distance = error_distance

        output_distance = (kP_distance * error_distance +
                           kI_distance * integral_distance +
                           kD_distance * derivative_distance)

        # Clamp the distance output to the maximum velocity
        if output_distance > max_velocity:
            output_distance = max_velocity
        elif output_distance < -max_velocity:
            output_distance = -max_velocity

        # ----- Heading Correction using IMU -----
        current_heading = Inertial21.rotation()
        error_heading = angle_diff(target_heading, current_heading)
        output_heading = kP_angle * error_heading

        # ----- Combine Distance and Heading Corrections -----
        # For a differential drive:
        left_output = output_distance + output_heading
        right_output = output_distance - output_heading

        # Clamp outputs to max_velocity limits
        left_output = max(-max_velocity, min(max_velocity, left_output))
        right_output = max(-max_velocity, min(max_velocity, right_output))

        # Determine drive direction based on desired distance
        if distance_inches >= 0:
            left_spin_direction = REVERSE
            right_spin_direction = FORWARD
        else:
            left_spin_direction = FORWARD
            right_spin_direction = REVERSE

        # ----- Command the Motors -----
        LeftMotors.set_velocity(left_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Right_front.set_velocity(right_output, PERCENT)

        LeftMotors.spin(left_spin_direction)
        Left_Front.spin(left_spin_direction)
        RightMotors.spin(right_spin_direction)
        Right_front.spin(right_spin_direction)

        wait(dt, SECONDS)

    # ----- Stop all motors when done -----
    LeftMotors.stop()
    Left_Front.stop()
    RightMotors.stop()
    Right_front.stop()


'''from MAIN_GB.main1 import *'''

# PID Turn Function

'''def pid_turn(target_angle, max_speed, timeout=3):
    Kp = 0.7   # Proportional Gain
    Ki = 0.0 # Integral Gain
    Kd = 4   # Derivative Gain

    integral = 0
    previous_error = 0
    threshold = 1.5  # Acceptable error in degrees
    start_time = brain.timer.time(SECONDS)

    # Reset IMU Heading
    Inertial21.set_rotation(0, DEGREES)
    
    while True:
        # Get current heading
        current_angle = Inertial21.rotation(DEGREES)
        error = target_angle - current_angle

        # If within acceptable range, stop
        if abs(error) < threshold or (brain.timer.time(SECONDS) - start_time) > timeout:
            break

        print(current_angle)

        # PID Calculations
        integral += error
        derivative = error - previous_error
        previous_error = error

        power = -(Kp * error) + (Ki * integral) + (Kd * derivative)
        power = max(min(power, max_speed), -max_speed)  # Limit speed


        # Apply power to motors for turning
        LeftMotors.set_velocity(power, PERCENT)
        RightMotors.set_velocity(power, PERCENT)
        Left_Front.set_velocity(power, PERCENT)
        Right_front.set_velocity(power, PERCENT)
        

        LeftMotors.spin(FORWARD)
        RightMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        Right_front.spin(FORWARD)

    # Stop motors after turn
    LeftMotors.stop()
    RightMotors.stop()
    Left_Front.stop()
    Right_front.stop()'''




# PID Turn Function
def pid_turn(target_angle, max_speed, timeout=3):
    Kp = 0.7   # Proportional Gain
    Ki = 0.0   # Integral Gain
    Kd = 4     # Derivative Gain

    integral = 0
    previous_error = 0
    threshold = 1.5  # Acceptable error in degrees
    start_time = brain.timer.time(SECONDS)

    # Reset IMU Heading
    Inertial21.set_rotation(0, DEGREES)

    while True:
        # Get current heading
        current_angle = Inertial21.rotation(DEGREES)
        error = target_angle - current_angle

        # Normalize the error to the shortest turn direction (-180 to 180 range)
        error = (error + 180) % 360 - 180

        # If within acceptable range, stop
        if abs(error) < threshold or (brain.timer.time(SECONDS) - start_time) > timeout:
            break

        print(f"Current Angle: {current_angle}, Error: {error}")

        # PID Calculations
        integral += error
        derivative = error - previous_error
        previous_error = error

        power = (Kp * error) + (Ki * integral) + (Kd * derivative)
        power = max(min(power, max_speed), -max_speed)  # Limit speed

        # Determine direction automatically
        if power > 0:  # Clockwise (right turn)
            LeftMotors.set_velocity(power, PERCENT)
            Left_Front.set_velocity(power, PERCENT)
            RightMotors.set_velocity(-power, PERCENT)  # Reverse right side
            Right_front.set_velocity(-power, PERCENT)
        else:  # Counterclockwise (left turn)
            LeftMotors.set_velocity(power, PERCENT)  # Reverse left side
            Left_Front.set_velocity(power, PERCENT)
            RightMotors.set_velocity(-power, PERCENT)  
            Right_front.set_velocity(-power, PERCENT)

        # Spin motors
        LeftMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        RightMotors.spin(FORWARD)
        Right_front.spin(FORWARD)

    # Stop motors after turn
    LeftMotors.stop()
    RightMotors.stop()
    Left_Front.stop()
    Right_front.stop()





def onauton_autonomous_0():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False
    Inertial21.calibrate()
    while Inertial21.is_calibrating():
        sleep(50)
    stop_initialize.broadcast()
    '''pid_turn(180,80)'''
    pid_drive(24, 50)
  



import time

# Autonomous Mode Options (Formatted for VEX Controller)
auto_modes = [
    "[NO AUTO]",
    "[RED LEFT RING]",
    "[BLUE RIGHT RING]",
    "[RED RIGHT STAKE]",
    "[BLUE LEFT STAKE]"
]

AutoSelect = 0  # Default to NO AUTO

def update_auto_display():
    """ Updates the controller screen with a simple animation effect """
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    
    # Display Auto Mode Name
    controller_1.screen.print(auto_modes[AutoSelect])

    # Show Navigation Controls
    controller_1.screen.set_cursor(2, 1)
    controller_1.screen.print("<< SELECT >>")

    # Rumble Feedback
    controller_1.rumble(".")

def fancy_scroll_effect():
    """ Creates a quick flashing effect when switching modes """
    for _ in range(2):
        controller_1.screen.clear_screen()
        wait(50, MSEC)
        update_auto_display()

def onevent_controller_1buttonL1_pressed_0():
    """ Scroll to the NEXT autonomous mode """
    global AutoSelect
    AutoSelect = (AutoSelect + 1) % len(auto_modes)
    controller_1.rumble(".-")  
    fancy_scroll_effect()

def onevent_controller_1buttonL2_pressed_0():
    """ Scroll to the PREVIOUS autonomous mode """
    global AutoSelect
    AutoSelect = (AutoSelect - 1) % len(auto_modes)
    controller_1.rumble("-..")  
    fancy_scroll_effect()

def when_started5():
    """ Initialize the auto selector with a clean display """
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("AUTO SELECT MODE")

    # Quick Flashing Effect
    for _ in range(3):
        controller_1.screen.print(".")
        wait(200, MSEC)

    update_auto_display()  # Show first selection
