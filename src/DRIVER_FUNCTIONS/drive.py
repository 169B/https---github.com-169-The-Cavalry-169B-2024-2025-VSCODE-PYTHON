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
        

