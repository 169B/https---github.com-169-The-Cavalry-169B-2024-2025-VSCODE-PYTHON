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
