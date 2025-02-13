
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
def slew_rate_limit(current, previous, max_delta=2):
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
        left_output = slew_rate_limit(desired_left_output, previous_left_output)
        right_output = slew_rate_limit(desired_right_output, previous_right_output)

        # Save current outputs for the next iteration
        previous_left_output = left_output
        previous_right_output = right_output

        # Set the motor velocities based on the computed outputs
        LeftMotors.set_velocity(left_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Right_front.set_velocity(right_output, PERCENT)

        # Spin the motors (adjust spin directions as needed for your drivetrain)
        LeftMotors.spin(REVERSE)
        Left_Front.spin(REVERSE)
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
