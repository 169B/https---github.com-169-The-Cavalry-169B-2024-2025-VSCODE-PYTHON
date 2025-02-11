
'''from MAIN_GB.main1 import *'''


# PID Turn Function
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
    Right_front.stop()

