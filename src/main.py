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
    """Updates the controller screen with a dynamic animation effect."""
    controller_1.screen.clear_screen()

    # Display selected Autonomous Mode with background color (if supported)
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("> {auto_modes[AutoSelect]} <")

    # Display Navigation Instructions with bold styling
    controller_1.screen.set_cursor(2, 1)
  
    controller_1.screen.print("<< SELECT >>")


    # Rumble feedback with more intensity for better interaction
    controller_1.rumble("...")

def fancy_scroll_effect():
    """Creates a cool flashing effect when switching modes."""
    for _ in range(2):
        controller_1.screen.clear_screen()
        wait(50, MSEC)
        update_auto_display()

def smooth_scroll_effect():
    """Smooth scrolling text effect when switching modes."""
    for i in range(len(auto_modes[AutoSelect])):
        controller_1.screen.set_cursor(1, 1)
        controller_1.screen.print(auto_modes[AutoSelect][:i + 1])
        wait(50, MSEC)
    update_auto_display()

def onevent_controller_1buttonL1_pressed_0():
    """Scroll to the NEXT autonomous mode."""
    global AutoSelect
    AutoSelect = (AutoSelect + 1) % len(auto_modes)
    controller_1.rumble("-.-")  # Short rumble for next mode
    smooth_scroll_effect()  # Apply smooth scrolling effect

def onevent_controller_1buttonL2_pressed_0():
    """Scroll to the PREVIOUS autonomous mode."""
    global AutoSelect
    AutoSelect = (AutoSelect - 1) % len(auto_modes)
    controller_1.rumble("-..")  # Short rumble for previous mode
    smooth_scroll_effect()  # Apply smooth scrolling effect

def when_started5():
    """Initialize the auto selector with a visually clean display."""
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("AUTO SELECT MODE")

    # Quick flashing effect to draw attention
    for _ in range(3):
        controller_1.screen.print(".")
        wait(200, MSEC)

    update_auto_display()  # Show initial auto mode'''





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
INTAKEF = False
INTAKER = False
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





'''def ondriver_drivercontrol_4():
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
        wait(5, MSEC)'''

def ondriver_drivercontrol_4():
    global rotation_15, Lady_Brown, controller_1

    # LADY BROWN INITIALIZATION
    Lady_Brown.spin_to_position(0, DEGREES)
    rotation_15.set_position(300, DEGREES)
    Lady_Brown.set_timeout(1, SECONDS)
    
    while True:
        # Move down (controller DOWN button)
        if controller_1.buttonDown.pressing() and rotation_15.position(DEGREES) > 198:
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)

            while rotation_15.position(DEGREES) > 276:
                wait(5, MSEC)

            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()

        # Move up (controller UP button)
        elif controller_1.buttonUp.pressing():
            Lady_Brown.set_velocity(100, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)

            while rotation_15.position(DEGREES) > 152:
                wait(5, MSEC)

            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()

        # Move back (controller LEFT button) with SAFEGUARD
        elif controller_1.buttonLeft.pressing():
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(REVERSE)

            start_time = time.time()  # Record start time
            while rotation_15.position(DEGREES) < 300:
                if time.time() - start_time > 2:  # If movement takes longer than 2 sec
                    print("⚠️ SAFEGUARD TRIGGERED: Movement timeout!")
                    break
                wait(5, MSEC)

            Lady_Brown.set_stopping(COAST)
            Lady_Brown.stop()
            rotation_15.set_position(300, DEGREES)

        wait(5, MSEC)  # Reduce CPU usage


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
    Inertial21.calibrate()
    while Inertial21.is_calibrating():
        sleep(50)

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

'''def when_started2():
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


def when_started2():
    global Blue, Red, optical_4, digital_out_e

    # Improved color sensing with debounce
    def detect_color(target_color):
        """ Waits until the optical sensor detects the target color reliably. """
        detection_count = 0  # Counter for consistent readings
        while detection_count < 5:  # Ensure stable detection before acting
            if optical_4.color() == target_color and optical_4.is_near_object():
                detection_count += 1
            else:
                detection_count = 0  # Reset counter if inconsistent
            wait(10, MSEC)

    while True:
        if Red:  # Sorting RED balls
            while Red:
                if optical_4.color() == Color.BLUE and optical_4.is_near_object():
                    digital_out_e.set(True)  # Activate mechanism
                    detect_color(Color.RED)  # Wait until next valid red object
                    digital_out_e.set(False)  # Deactivate mechanism
                wait(10, MSEC)

        elif Blue:  # Sorting BLUE balls
            while Blue:
                if optical_4.color() == Color.RED and optical_4.is_near_object():
                    digital_out_e.set(True)  # Activate mechanism
                    detect_color(Color.BLUE)  # Wait until next valid blue object
                    digital_out_e.set(False)  # Deactivate mechanism
                wait(10, MSEC)

        wait(10, MSEC)  # Reduce CPU usage


def when_started3():
    global Intake_running, intake

    # Anti-jam system with smart detection
    def anti_jam():
        """ Detects intake stall and reverses to clear jams. """
        jam_timer = 0
        while Intake_running:
            if not intake.is_spinning():  # Detects jam if motor is not moving
                jam_timer += 1
                if jam_timer > 10:  # If stalled for 10 iterations (~50ms each)
                    intake.spin(REVERSE)  # Reverse intake
                    wait(500, MSEC)  # Allow jam clearing
                    intake.spin(FORWARD)  # Resume normal operation
                    jam_timer = 0  # Reset jam detection timer
            else:
                jam_timer = 0  # Reset if intake moves normally
            wait(5, MSEC)

    while True:
        anti_jam()
        wait(10, MSEC)  # Reduce unnecessary CPU usage'''
        





def Move_In_direction_Degree_Speed(Move_In_direction_Degree_Speed__Degree, Move_In_direction_Degree_Speed__Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # LINKED WITH CURRENT PID FOR MOVEMENT VELOCITY
    if Move_In_direction_Degree_Speed__Degree > 0:
        RightMotors.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        Right_front.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        LeftMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Left_Front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
    else:
        RightMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Right_front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        LeftMotors.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        Left_Front.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)


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
        direction = (Kp * Proportional + (Ki * integral + Kd * derivitive))
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


import math



# PID Constants for Distance Control
kP_distance = 1.0
kI_distance = 0.0  
kD_distance = 0.1

# PID Constant for Heading Correction
kP_heading = 0.05

def pid_drive(distance_inches, max_velocity_percent, timeout=20.0):

    LeftMotors.set_stopping(BRAKE)
    RightMotors.set_stopping(BRAKE)
    Left_Front.set_stopping(BRAKE)
    Right_front.set_stopping(BRAKE)
    """
    Drives the robot forward for a given distance (in inches) with PID control 
    and IMU-based heading correction.

    Args:
        distance_inches (float): Target distance to move (in inches).
        max_velocity_percent (float): Maximum motor speed (0-100%).
        timeout (float): Maximum time allowed for movement (seconds).
    """

    # Reset motor encoders
    LeftMotors.set_position(0, DEGREES)
    RightMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)

    # Capture the starting heading from the IMU
    target_heading = Inertial21.rotation()


    integral_distance = 0.0
    last_error_distance = 0.0
    start_time = brain.timer.time(SECONDS)  # Use Brain's timer

    while True:

        # Calculate error in distance (in encoder ticks)
        error_distance = distance_inches - (((RightMotors.position(DEGREES)/360)*math.pi*2.75)+((Right_front.position(DEGREES)/360)*math.pi*2.75)+((LeftMotors.position(DEGREES)/360)*math.pi*2.75)+((Left_Front.position(DEGREES)/360)*math.pi*2.75)/4)

        # Break if we're within tolerance or timeout has been exceeded
        if abs(error_distance) < 2 or (brain.timer.time(SECONDS) - start_time) > timeout:
            break
        controller_1.screen.set_cursor(1,1)
        wait(0.2,SECONDS)
        controller_1.screen.clear_screen()
        controller_1.screen.print(error_distance)

        # Distance PID calculations
        integral_distance += error_distance
        derivative_distance = error_distance - last_error_distance
        last_error_distance = error_distance

        pid_output = (kP_distance * error_distance +
                      kI_distance * integral_distance +
                      kD_distance * derivative_distance)

        # Clamp the output to the maximum allowed velocity
        pid_output = max(-max_velocity_percent, min(max_velocity_percent, pid_output))
        
        # Heading correction using IMU
        current_heading = Inertial21.rotation()
        error_heading = target_heading - current_heading
        heading_correction = kP_heading * error_heading

        # Combine outputs: Add heading correction to the left side and subtract from the right
        left_output = pid_output + heading_correction
        right_output = pid_output - heading_correction

        # Clamp motor outputs to the maximum allowed velocity
        left_output = max(-max_velocity_percent, min(max_velocity_percent, left_output))
        right_output = max(-max_velocity_percent, min(max_velocity_percent, right_output))

        # Set motor velocities
        LeftMotors.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        Right_front.set_velocity(right_output, PERCENT)

        # Spin motors
        LeftMotors.spin(REVERSE)
        Left_Front.spin(REVERSE)
        RightMotors.spin(FORWARD)
        Right_front.spin(FORWARD)

    # Stop motors when the movement is complete
    RightMotors.stop()
    Right_front.stop()
    LeftMotors.stop()
    Left_Front.stop()

'''def pid_turn(target_heading, max_velocity, momentum):
    global Inertial21, RightMotors, Right_front, LeftMotors, Left_Front
    
    # PID Constants (Tweak for best performance)
    Kp = 0.35  # Increased proportional gain for faster response
    Ki = 0.00005  # Further reduced integral gain to minimize overshoot
    Kd = 0.05  # Reduced derivative gain to avoid excessive damping
    Kp = 0.5  # Increased proportional gain for faster response
    Ki = 0.005  # Further reduced integral gain to minimize overshoot
    Kd = 0.45  # Reduced derivative gain to avoid excessive damping

    # Get current heading (DO NOT RESET IMU)
    start_heading = Inertial21.rotation(DEGREES)
    target = start_heading + (target_heading-2)  # Adjust for relative turning

    # Initialize PID variables
    prev_error = 0
    integral = 0
    import time
    start_time = time.time()  # Timeout start time

    while True:
        # Calculate error (how far from target)
        error = target - Inertial21.rotation(DEGREES)

        # Stop if within momentum threshold
        if abs(error) < 2:  
            break

        # Timeout safety to prevent infinite loops
        if time.time() - start_time > 50:  # 5-second timeout
            break

        # PID calculations
        integral += error  # Accumulate error over time
        derivative = error - prev_error  # Change in error
        prev_error = error  # Store current error

        # Compute PID output
        speed = (Kp * error) + (Ki * integral) + (Kd * derivative)

        # Slow down near the target to prevent overshoot (steeper exponential decay)
        speed *= max(0.2, 1 - (abs(error) / target_heading) ** 2)  # Steeper decay

        # Limit speed to max_velocity and reasonable minimum
        speed = max(min(speed, max_velocity), 20)  # Minimum speed = 20%

        # Small Deadband to stop oscillation (error < 2 degrees)
        if abs(error) < 2:
            speed = 0  # Stop robot to prevent oscillation

        # Debugging feedback on controller screen
        controller_1.screen.clear_screen()
        wait(0.0002, SECONDS)
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(error)

        # Apply speed to motors for turning
        if error > 0:  # Turn RIGHT
            RightMotors.set_velocity(speed, PERCENT)
            Right_front.set_velocity(speed, PERCENT)
            LeftMotors.set_velocity(speed, PERCENT)
            Left_Front.set_velocity(speed, PERCENT)

            RightMotors.spin(REVERSE)  # Reverse for right turn
            Right_front.spin(REVERSE)
            LeftMotors.spin(REVERSE)  # Forward for right turn
            Left_Front.spin(REVERSE)
        else:  # Turn LEFT
            RightMotors.set_velocity(speed, PERCENT)
            Right_front.set_velocity(speed, PERCENT)
            LeftMotors.set_velocity(speed, PERCENT)
            Left_Front.set_velocity(speed, PERCENT)

            RightMotors.spin(FORWARD)  # Forward for left turn
            Right_front.spin(FORWARD)
            LeftMotors.spin(FORWARD)  # Reverse for left turn
            Left_Front.spin(FORWARD)

        wait(10, MSEC)  # Small delay for smooth updates

    # Stop motors after reaching target
    RightMotors.stop()
    LeftMotors.stop()
    Right_front.stop()
    Left_Front.stop()'''


def pid_turn(target_heading, max_velocity):
    global Inertial21, RightMotors, Right_front, LeftMotors, Left_Front
    
    # PID Constants (Adjusted for faster turns)
    Kp = 0.65  # Increased proportional gain by 20% for faster response
    Kd = 0.43  # Derivative gain (kept the same for stability)

    # Get current heading (DO NOT RESET IMU)
    start_heading = Inertial21.rotation(DEGREES)
    target = start_heading + (target_heading - 5)  # Adjust for relative turning

    # Initialize PID variables
    prev_error = 0
    import time
    start_time = time.time()  # Timeout start time

    while True:
        # Calculate error (how far from target)
        error = target - Inertial21.rotation(DEGREES)

        # If the error is small enough, stop
        if abs(error) < 0.5:  # Close enough threshold for accuracy
            break

        # Timeout safety to prevent infinite loops
        if time.time() - start_time > 5:  # Timeout after 5 seconds
            break

        # PID calculations
        derivative = error - prev_error  # Change in error
        prev_error = error  # Store current error

        # Compute PID output
        speed = (Kp * error) + (Kd * derivative)

        # Adaptive speed adjustment to avoid overshooting (lower speed as close to target)
        speed *= max(0.1, 1 - (abs(error) / target_heading) ** 2)

        # Limit speed to max_velocity and reasonable minimum
        speed = max(min(speed, max_velocity), 15)  # Minimum speed = 15%

        # Apply speed to motors for turning
        if error > 0:  # Turn RIGHT
            RightMotors.set_velocity(speed, PERCENT)
            Right_front.set_velocity(speed, PERCENT)
            LeftMotors.set_velocity(speed, PERCENT)
            Left_Front.set_velocity(speed, PERCENT)

            RightMotors.spin(REVERSE)  # Reverse for right turn
            Right_front.spin(REVERSE)
            LeftMotors.spin(REVERSE)  # Forward for right turn
            Left_Front.spin(REVERSE)
        else:  # Turn LEFT
            RightMotors.set_velocity(speed, PERCENT)
            Right_front.set_velocity(speed, PERCENT)
            LeftMotors.set_velocity(speed, PERCENT)
            Left_Front.set_velocity(speed, PERCENT)

            RightMotors.spin(FORWARD)  # Forward for left turn
            Right_front.spin(FORWARD)
            LeftMotors.spin(FORWARD)  # Reverse for left turn
            Left_Front.spin(FORWARD)

        wait(10, MSEC)  # Small delay for smoother updates

    # Stop motors after reaching target
    RightMotors.stop()
    LeftMotors.stop()
    Right_front.stop()
    Left_Front.stop()























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
    '''driver_control_task_0.stop()'''
    driver_control_task_1.stop()
    driver_control_task_2.stop()
    '''driver_control_task_3.stop()'''
    driver_control_task_4.stop()


# register the competition functions
competition = Competition( vexcode_driver_function, vexcode_auton_function )

# system event handlers
'''controller_1.axis2.changed(onevent_controller_1axis2Changed_0)
controller_1.axis3.changed(onevent_controller_1axis3Changed_0)'''
stop_initialize(onevent_stop_initialize_0)
'''controller_1.buttonL1.pressed(onevent_controller_1buttonL1_pressed_0)
controller_1.buttonL2.pressed(onevent_controller_1buttonL2_pressed_0)'''
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

'''ws2 = Thread( when_started2 )
ws3 = Thread( when_started3 )'''
ws4 = Thread( when_started4 )
'''ws5 = Thread( when_started5 )'''
import time




def onauton_autonomous_0():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False
    stop_initialize.broadcast()
    # AUTO SELECT
    RED_LEFT_RING()



def RED_LEFT_RING():
    return


    '''pid_drive(-50, 80)
    digital_out_b.set(True)
    intake.spin(FORWARD)
    pid_turn(180, 100)
    wait(0.5, SECONDS)'''
    
   
   





import math

# Function to limit how fast the output can change (slew rate limiting)
def slew_rate_limit(current, previous, max_delta=5):
    delta = current - previous
    if abs(delta) > max_delta:
        return previous + max_delta * (1 if delta > 0 else -1)
    return current

def cubic_scaling(value):
    """Applies cubic scaling to joystick input for smoother control."""
    normalized = value / 100.0
    return (normalized ** 3) * 100  # Scale back to percentage

def ondriver_drivercontrol_1():
    global Left_Axis, Right_Axis, previous_left_output, previous_right_output

    max_velocity = 100  # Max motor speed (percent)

    # Initialize previous outputs for slew rate limiting
    previous_left_output = 0
    previous_right_output = 0

    # Set motors to coast for smoother motion
    LeftMotors.set_stopping(COAST)
    Left_Front.set_stopping(COAST)
    RightMotors.set_stopping(COAST)
    Right_front.set_stopping(COAST)

    while True:
        # Read joystick values
        Left_Axis = controller_1.axis3.position()
        Right_Axis = controller_1.axis2.position()

        # Apply cubic scaling for smoother control
        desired_left_output = cubic_scaling(Left_Axis)
        desired_right_output = cubic_scaling(Right_Axis)

        # Apply slew rate limiting to smooth out rapid changes
        left_output = slew_rate_limit(desired_left_output, previous_left_output)
        right_output = slew_rate_limit(desired_right_output, previous_right_output)

        # Save current outputs for the next iteration
        previous_left_output = left_output
        previous_right_output = right_output

        # Set motor velocities and spin the motors
        LeftMotors.set_velocity(left_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Right_front.set_velocity(right_output,


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
            wait(0.2, SECONDS)
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
            wait(0.2, SECONDS)
        wait(5, MSEC)
        
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
        



def draw_speedometer():
    brain.screen.clear_screen()

    # Draw speedometer outline
    brain.screen.draw_circle(160, 120, 50)  # Outer circle
    brain.screen.print(140, 170, "Speed")

    # Get motor speed (0-100%)
    speed = RightMotors.velocity(PERCENT)
    
    # Convert speed to angle (-90° to 90°)
    angle = -90 + (speed * 1.8)

    # Calculate needle position
    needle_x = 160 + 40 * math.cos(angle)
    needle_y = 120 + 40 * math.sin(angle)

    # Draw speedometer needle
    brain.screen.draw_line(160, 120, needle_x, needle_y)

    # Render updated screen
    brain.screen.render()



'''def when_started2():
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


def when_started2():
    global Blue, Red, optical_4, digital_out_e

    # Improved color sensing with debounce
    def detect_color(target_color):
        """ Waits until the optical sensor detects the target color reliably. """
        detection_count = 0  # Counter for consistent readings
        while detection_count < 5:  # Ensure stable detection before acting
            if optical_4.color() == target_color and optical_4.is_near_object():
                detection_count += 1
            else:
                detection_count = 0  # Reset counter if inconsistent
            wait(10, MSEC)

    while True:
        if Red:  # Sorting RED balls
            while Red:
                if optical_4.color() == Color.BLUE and optical_4.is_near_object():
                    digital_out_e.set(True)  # Activate mechanism
                    detect_color(Color.RED)  # Wait until next valid red object
                    digital_out_e.set(False)  # Deactivate mechanism
                wait(10, MSEC)

        elif Blue:  # Sorting BLUE balls
            while Blue:
                if optical_4.color() == Color.RED and optical_4.is_near_object():
                    digital_out_e.set(True)  # Activate mechanism
                    detect_color(Color.BLUE)  # Wait until next valid blue object
                    digital_out_e.set(False)  # Deactivate mechanism
                wait(10, MSEC)

        wait(10, MSEC)  # Reduce CPU usage


def when_started3():
    global Intake_running, intake

    # Anti-jam system with smart detection
    def anti_jam():
        """ Detects intake stall and reverses to clear jams. """
        jam_timer = 0
        while Intake_running:
            if not intake.is_spinning():  # Detects jam if motor is not moving
                jam_timer += 1
                if jam_timer > 10:  # If stalled for 10 iterations (~50ms each)
                    intake.spin(REVERSE)  # Reverse intake
                    wait(500, MSEC)  # Allow jam clearing
                    intake.spin(FORWARD)  # Resume normal operation
                    jam_timer = 0  # Reset jam detection timer
            else:
                jam_timer = 0  # Reset if intake moves normally
            wait(5, MSEC)

    while True:
        anti_jam()
        wait(10, MSEC)  # Reduce unnecessary CPU usage'''
        



