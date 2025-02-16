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

