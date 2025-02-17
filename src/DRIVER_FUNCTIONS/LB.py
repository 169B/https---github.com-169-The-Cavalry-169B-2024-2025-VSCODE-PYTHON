


'''def ondriver_drivercontrol_4():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # LADY BROWN MACROS AND INIT
    Lady_Brown.spin_to_position(0, DEGREES)
    rotation_15.set_position(300, DEGREES)
    Lady_Brown.set_timeout(1, SECONDS)
    Lady_Brown.set_velocity(40, PERCENT)
    while True:
        
        if controller_1.buttonDown.pressing() and 198 < rotation_15.position(DEGREES):
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)
            while not rotation_15.position(DEGREES) < 276:
                wait(5, MSEC)
            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()
        if controller_1.buttonUp.pressing():
            Lady_Brown.set_velocity(100, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)
            while not rotation_15.position(DEGREES) < 152:
                wait(5, MSEC)
            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()
        if controller_1.buttonLeft.pressing():
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(REVERSE)
            while not 300 < rotation_15.position(DEGREES):
                wait(5, MSEC)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.stop()
        wait(5, MSEC)'''

def ondriver_drivercontrol_4():
    global rotation_15, Lady_Brown, controller_1

    # LADY BROWN INITIALIZATION
    Lady_Brown.spin_to_position(0, DEGREES)
    rotation_15.set_position(300, DEGREES)
    Lady_Brown.set_timeout(1, SECONDS)
    
    while True:
        # Move down (controller DOWN button)
        if controller_1.buttonDown.pressing() and rotation_15.position(DEGREES) > 198:
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)

            while rotation_15.position(DEGREES) > 276:
                wait(5, MSEC)

            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()

        # Move up (controller UP button)
        elif controller_1.buttonUp.pressing():
            Lady_Brown.set_velocity(100, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)

            while rotation_15.position(DEGREES) > 152:
                wait(5, MSEC)

            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()

        # Move back (controller LEFT button) with SAFEGUARD
        elif controller_1.buttonLeft.pressing():
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(REVERSE)

            start_time = time.time()  # Record start time
            while rotation_15.position(DEGREES) < 300:
                if time.time() - start_time > 2:  # If movement takes longer than 2 sec
                    print("⚠️ SAFEGUARD TRIGGERED: Movement timeout!")
                    break
                wait(5, MSEC)

            Lady_Brown.set_stopping(COAST)
            Lady_Brown.stop()
            rotation_15.set_position(300, DEGREES)

        wait(5, MSEC)  # Reduce CPU usage

