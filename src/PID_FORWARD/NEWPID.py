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
