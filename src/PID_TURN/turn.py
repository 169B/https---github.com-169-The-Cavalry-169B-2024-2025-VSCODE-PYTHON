
'''from MAIN_GB.main1 import *'''

'''def turn_heading_velocity_momentum(turn_heading_velocity_momentum__heading, turn_heading_velocity_momentum__velocity, turn_heading_velocity_momentum__momentum):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CURRENT ACCURATE TURN USING IMU
    Inertial21.set_rotation(0, DEGREES)
    if turn_heading_velocity_momentum__heading > Inertial21.rotation(DEGREES):
        while turn_heading_velocity_momentum__heading - turn_heading_velocity_momentum__momentum > Inertial21.rotation(DEGREES):
            RightMotors.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            Right_front.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            LeftMotors.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            Left_Front.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            RightMotors.spin(REVERSE)
            Right_front.spin(REVERSE)
            LeftMotors.spin(REVERSE)
            Left_Front.spin(REVERSE)
            wait(5, MSEC)
    else:
        while turn_heading_velocity_momentum__heading + turn_heading_velocity_momentum__momentum < Inertial21.rotation(DEGREES):
            RightMotors.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            Right_front.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            LeftMotors.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            Left_Front.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
            RightMotors.spin(FORWARD)
            Right_front.spin(FORWARD)
            LeftMotors.spin(FORWARD)
            Left_Front.spin(FORWARD)
            wait(5, MSEC)
    RightMotors.stop()
    LeftMotors.stop()
    Right_front.stop()
    Left_Front.stop()
    turn1 = 1'''



    # Turn Function
def turn(target_angle, max_speed):
    Kp = 0.08  # Proportional Gain
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
        if abs(error) < threshold or (brain.timer.time(SECONDS) - start_time) > 3:
            break

        # Proportional controller
        power = Kp * error
        power = max(min(power, max_speed), -max_speed)  # Limit speed

        # Apply power to motors for turning
        LeftMotors.set_velocity(power, PERCENT)
        RightMotors.set_velocity(power, PERCENT)
        Left_Front.set_velocity(power, PERCENT)
        Right_front.set_velocity(power, PERCENT)

        # Spin motors forward
        LeftMotors.spin(FORWARD)
        RightMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        Right_front.spin(FORWARD)

    # Stop motors after turn
    LeftMotors.stop()
    RightMotors.stop()
    Left_Front.stop()
    Right_front.stop()




    def turn_heading_velocity_momentum(turn_heading_velocity_momentum__heading, turn_heading_velocity_momentum__velocity, turn_heading_velocity_momentum__momentum):
 

        # CURRENT ACCURATE TURN USING IMU
        Inertial21.set_rotation(0, DEGREES)

        # Calculate the desired rotation direction
        rotation_direction = 1 if turn_heading_velocity_momentum__heading > Inertial21.rotation(DEGREES) else -1

        # Calculate the desired turn angle
        turn_angle = abs(turn_heading_velocity_momentum__heading - Inertial21.rotation(DEGREES))

        # Calculate the desired number of iterations
        num_iterations = int(turn_angle / 5)  # 5 degrees per iteration

        # Set the motor velocities
        RightMotors.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
        Right_front.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
        LeftMotors.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)
        Left_Front.set_velocity(turn_heading_velocity_momentum__velocity, PERCENT)

        # Set the motor direction
        RightMotors.spin(FORWARD if rotation_direction == 1 else REVERSE)
        Right_front.spin(FORWARD if rotation_direction == 1 else REVERSE)
        LeftMotors.spin(FORWARD if rotation_direction == -1 else REVERSE)
        Left_Front.spin(FORWARD if rotation_direction == -1 else REVERSE)

        # Perform the turn
        for _ in range(num_iterations):
            wait(5, MSEC)

        # Stop the motors
        RightMotors.stop()
        LeftMotors.stop()
        Right_front.stop()
        Left_Front.stop()