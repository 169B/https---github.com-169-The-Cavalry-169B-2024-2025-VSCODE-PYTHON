import math

# Function to limit how fast the output can change (slew rate limiting)
def slew_rate_limit(current, previous, max_delta=9):
    delta = current - previous
    if abs(delta) > max_delta:
        return previous + max_delta * (1 if delta > 0 else -1)
    return current

def ondriver_drivercontrol_1():
    global Left_Axis, Right_Axis, previous_left_output, previous_right_output, max_velocity, dead_zone_range

    dead_zone_range = 10  # Adjust this value to set the dead zone range
    max_velocity = 100  # Maximum motor speed (percent)

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

        # Apply dead zone filtering
    

        # Normalize inputs and apply cubic scaling for smoother control
        left_normalized = Left_Axis / 100.0
        right_normalized = Right_Axis / 100.0

        # Apply cubic scaling
        left_cubic = left_normalized ** 3
        right_cubic = right_normalized ** 3

        # Calculate the desired motor outputs
        desired_left_output = left_cubic * max_velocity
        desired_right_output = right_cubic * max_velocity

        # Apply slew rate limiting
        left_output = slew_rate_limit(desired_left_output, previous_left_output)
        right_output = slew_rate_limit(desired_right_output, previous_right_output)

        # Save current outputs for the next iteration
        previous_left_output = left_output
        previous_right_output = right_output

        # Set motor velocities and spin the motors
        LeftMotors.set_velocity(left_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Right_front.set_velocity(right_output, PERCENT)

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
    global Intake_Control, DOon, intake, INTAKEF, INTAKER

    while True:
        if controller_1.buttonR1.pressing():
            # Toggle intake forward
            if not INTAKEF:
                intake.set_velocity(80, PERCENT)
                intake.spin(FORWARD)
                INTAKEF = True
            else:
                intake.stop()
                INTAKEF = False

        elif controller_1.buttonR2.pressing():
            # Toggle intake backward
            if INTAKER:
                intake.set_velocity(80, PERCENT)
                intake.spin(REVERSE)
            else:
                intake.stop()
            INTAKER = False

        wait(5, MSEC)
