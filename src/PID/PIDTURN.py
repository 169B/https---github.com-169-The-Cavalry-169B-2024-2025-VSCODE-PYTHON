
'''from MAIN_GB.main1 import *'''


'''# PID Turn Function
def pid_turn(target_angle, max_speed, timeout=3):
    Kp = 5   # Proportional Gain
    Ki = 0.2 # Integral Gain
    Kd = 3   # Derivative Gain

    integral = 0
    previous_error = 0
    threshold = 1  # Acceptable error in degrees
    start_time = brain.timer.time(SECONDS)

    # Reset IMU Heading
    Inertial21.set_heading(0, DEGREES)
    
    while True:
        # Get current heading
        current_angle = Inertial21.heading(DEGREES)
        error = target_angle - current_angle

        # If within acceptable range, stop
        if abs(error) < threshold or (brain.timer.time(SECONDS) - start_time) > timeout:
            break

        # PID Calculations
        integral += error
        derivative = error - previous_error
        previous_error = error

        power = (Kp * error) + (Ki * integral) + (Kd * derivative)
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

'''def pid_turn(target_angle, max_speed, timeout=3):
    
    # PID gains
    Kp = 0.1   # Proportional Gain
    Ki = 0 # Integral Gain
    Kd = 0   # Derivative Gain

    # Integral and derivative variables
    integral = 0
    previous_error = 0

    # Threshold for acceptable error
    threshold = 1  # Acceptable error in degrees

    # Start time
    start_time = brain.timer.time(SECONDS)

    # Reset IMU heading
    try:
        Inertial21.set_heading(0, DEGREES)
    except Exception as e:
        print("Error resetting IMU heading:")

    while True:
        # Get current heading
        try:
            current_angle = Inertial21.heading(DEGREES)
        except Exception as e:
            print("Error getting current heading:")
            break

        # Calculate error
        error = target_angle - current_angle

        # If within acceptable range, stop
        if abs(error) < threshold or (brain.timer.time(SECONDS) - start_time) > timeout:
            break

        # PID calculations
        integral += error
        derivative = error - previous_error
        previous_error = error

        # Calculate power
        power = (Kp * error) + (Ki * integral) + (Kd * derivative)
        power = max(min(power, max_speed), -max_speed)  # Limit speed

        # Apply power to motors for turning
        try:
            LeftMotors.set_velocity(power, PERCENT)
            RightMotors.set_velocity(power, PERCENT)
            Left_Front.set_velocity(power, PERCENT)
            Right_front.set_velocity(power, PERCENT)
        except Exception as e:
            print("Error setting motor velocity:")

        try:
            LeftMotors.spin(FORWARD)
            RightMotors.spin(FORWARD)
            Left_Front.spin(FORWARD)
            Right_front.spin(FORWARD)
        except Exception as e:
            print("Error spinning motors:")

    # Stop motors after turn
    try:
        LeftMotors.stop()
        RightMotors.stop()
        Left_Front.stop()
        Right_front.stop()
    except Exception as e:
        print("Error stopping motors:")'''

def angle_diff(target, current):
    """
    Computes the shortest (signed) difference between two angles.
    Returns a value between -180 and 180 degrees.
    """
    diff = (target - current + 180) % 360 - 180
    return diff

def pid_turn(degreeChange, timeout=4.0):
    """
    Performs an in-place turn by a relative amount of 'degreeChange' degrees.
    This function uses a PID loop that:
      - Reads the current angle from the inertial sensor.
      - Uses the sensor’s measured angular velocity for a more accurate derivative.
      - Commands the motors using velocity control.
    
    Adjust the PID constants (kP, kI, kD) as needed.
    """
  

    # Get the current heading and compute the target heading (normalized to 0–359°)
    currentHeading = Inertial21.rotation() % 360
    targetHeading = (currentHeading + degreeChange) % 360

    # ------------------------------
    # PID Tuning Constants (tweak as needed)
    kP = 0.3
    kI = 0
    # Instead of computing derivative numerically, we use the IMU’s measured angular velocity.
    # For a constant target, the derivative of error = - (angular velocity).
    kD = 5
    # ------------------------------

    integral = 0.0
    dt = 0.02           # Loop time (20 ms)
    tolerance = 3     # Acceptable error in degrees
    max_velocity =50 # Maximum motor velocity (in percent)

    start_time = brain.timer.time(SECONDS)

    while True:
        # Read current heading from the IMU
        currentHeading = Inertial21.rotation() % 360

        # Compute error (shortest angle difference)
        error = angle_diff(targetHeading, currentHeading)
        if abs(error) < tolerance:
            break

        # Use the IMU’s angular velocity (in deg/sec) as the derivative feedback.
        # For a constant target, the derivative of error equals - (angular velocity).
        angular_velocity = Inertial21.gyro_rate(XAXIS,VelocityUnits.DPS)  # Assumes degrees per second
        derivative = -angular_velocity

        # Accumulate the error for the integral term.
        integral += error * dt

        # Compute the PID output as a velocity command.
        # (Units here are "percent" for motor velocity.)
        pid_output = kP * error + kI * integral + kD * derivative

        # Clamp the output so it does not exceed the maximum allowed velocity.
        if pid_output > max_velocity:
            pid_output = max_velocity
        elif pid_output < -max_velocity:
            pid_output = -max_velocity

        # --- Command the Motors using velocity control ---
        # For an in-place turn:
        #   - Left side motors: spin FORWARD with the computed velocity.
        #   - Right side motors: spin FORWARD with the negative of that velocity.
        LeftMotors.set_velocity(pid_output, PERCENT)
        Left_Front.set_velocity(pid_output, PERCENT)
        RightMotors.set_velocity(pid_output, PERCENT)
        Right_front.set_velocity(pid_output, PERCENT)

        # Spin the motors. (They will run at the velocities set above.)
        LeftMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        RightMotors.spin(FORWARD)
        Right_front.spin(FORWARD)

 

        # Exit if we exceed the allowed timeout.
        if brain.timer.time(SECONDS) - start_time > timeout:
            break

    # Stop all drive motors (using the default brake type)
    LeftMotors.stop()
    Left_Front.stop()
    RightMotors.stop()
    Right_front.stop()

# ------------------------------------------------------------------------------
# Example usage:
# To command a 90° turn (relative to the current heading), simply call: