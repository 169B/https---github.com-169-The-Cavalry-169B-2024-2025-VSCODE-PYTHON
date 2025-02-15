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
    global Intake_Control, DOon, intake

    while True:
        if controller_1.buttonR1.pressing():
            # Toggle intake forward
            if not DOon:
                intake.set_velocity(80, PERCENT)
                intake.spin(FORWARD)
                DOon = True
            else:
                intake.stop()
                DOon = False

        elif controller_1.buttonR2.pressing():
            # Toggle intake backward
            if DOon:
                intake.set_velocity(80, PERCENT)
                intake.spin(REVERSE)
            else:
                intake.stop()
            DOon = False

        wait(5, MSEC)
