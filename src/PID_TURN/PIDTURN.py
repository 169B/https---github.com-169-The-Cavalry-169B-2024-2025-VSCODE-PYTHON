

def pid_turn(target_heading, max_velocity, momentum):
    global Inertial21, RightMotors, Right_front, LeftMotors, Left_Front
    
    Inertial21.set_rotation(0, DEGREES)  # Reset IMU

    while True:
        # Calculate remaining degrees to turn
        error = target_heading - Inertial21.rotation(DEGREES)

        # Stop if within momentum threshold
        if abs(error) < momentum:
            break

        # Gradually reduce speed as robot approaches target
        speed = max_velocity * (abs(error) / max(abs(target_heading), 1))  # Prevent division by zero
        speed = max(speed, 5)  # Ensure a small minimum speed to prevent stalling
        controller_1.screen.clear_screen()
        wait(0.02, SECONDS)
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(error)

        # Apply speed to motors
        RightMotors.set_velocity(speed, PERCENT)
        Right_front.set_velocity(speed, PERCENT)
        LeftMotors.set_velocity(speed, PERCENT)
        Left_Front.set_velocity(speed, PERCENT)

        # Set direction
        if error > 0:
            RightMotors.spin(REVERSE)
            Right_front.spin(REVERSE)
            LeftMotors.spin(FORWARD)
            Left_Front.spin(FORWARD)
        else:
            RightMotors.spin(FORWARD)
            Right_front.spin(FORWARD)
            LeftMotors.spin(REVERSE)
            Left_Front.spin(REVERSE)

        wait(10, MSEC)  # Small delay for smooth updates

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
    driver_control_task_0.stop()
    driver_control_task_1.stop()
    driver_control_task_2.stop()
    driver_control_task_3.stop()
    driver_control_task_4.stop()


# register the competition functions
competition = Competition( vexcode_driver_function, vexcode_auton_function )

# system event handlers
controller_1.axis2.changed(onevent_controller_1axis2Changed_0)
controller_1.axis3.changed(onevent_controller_1axis3Changed_0)
stop_initialize(onevent_stop_initialize_0)
controller_1.buttonL1.pressed(onevent_controller_1buttonL1_pressed_0)
controller_1.buttonL2.pressed(onevent_controller_1buttonL2_pressed_0)
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

ws2 = Thread( when_started2 )
ws3 = Thread( when_started3 )
ws4 = Thread( when_started4 )
ws5 = Thread( when_started5 )
