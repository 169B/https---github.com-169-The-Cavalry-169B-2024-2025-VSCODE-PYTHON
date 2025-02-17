

'''def when_started2():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    while True:
        if Red:
            while Red:
                if optical_4.color() == Color.BLUE and optical_4.is_near_object():
                    digital_out_e.set(True)
                    while not (optical_4.color() == Color.RED and optical_4.is_near_object()):
                        wait(5, MSEC)
                    digital_out_e.set(False)
                wait(5, MSEC)
        elif Blue:
            while Blue:
                if optical_4.color() == Color.RED and optical_4.is_near_object():
                    digital_out_e.set(True)
                    while not (optical_4.color() == Color.BLUE and optical_4.is_near_object()):
                        wait(5, MSEC)
                    digital_out_e.set(False)
                wait(5, MSEC)
        else:
            wait(5, MSEC)


def when_started3():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # SENSING FOR ANTI-OBSTRUTION - GLOBAL
    while True:
        while Intake_running:
            if not intake.is_spinning():
                intake.spin(REVERSE)
                wait(0.5, SECONDS)
            wait(5, MSEC)
        wait(5, MSEC)


def when_started2():
    global Blue, Red, optical_4, digital_out_e

    # Improved color sensing with debounce
    def detect_color(target_color):
        """ Waits until the optical sensor detects the target color reliably. """
        detection_count = 0  # Counter for consistent readings
        while detection_count < 5:  # Ensure stable detection before acting
            if optical_4.color() == target_color and optical_4.is_near_object():
                detection_count += 1
            else:
                detection_count = 0  # Reset counter if inconsistent
            wait(10, MSEC)

    while True:
        if Red:  # Sorting RED balls
            while Red:
                if optical_4.color() == Color.BLUE and optical_4.is_near_object():
                    digital_out_e.set(True)  # Activate mechanism
                    detect_color(Color.RED)  # Wait until next valid red object
                    digital_out_e.set(False)  # Deactivate mechanism
                wait(10, MSEC)

        elif Blue:  # Sorting BLUE balls
            while Blue:
                if optical_4.color() == Color.RED and optical_4.is_near_object():
                    digital_out_e.set(True)  # Activate mechanism
                    detect_color(Color.BLUE)  # Wait until next valid blue object
                    digital_out_e.set(False)  # Deactivate mechanism
                wait(10, MSEC)

        wait(10, MSEC)  # Reduce CPU usage


def when_started3():
    global Intake_running, intake

    # Anti-jam system with smart detection
    def anti_jam():
        """ Detects intake stall and reverses to clear jams. """
        jam_timer = 0
        while Intake_running:
            if not intake.is_spinning():  # Detects jam if motor is not moving
                jam_timer += 1
                if jam_timer > 10:  # If stalled for 10 iterations (~50ms each)
                    intake.spin(REVERSE)  # Reverse intake
                    wait(500, MSEC)  # Allow jam clearing
                    intake.spin(FORWARD)  # Resume normal operation
                    jam_timer = 0  # Reset jam detection timer
            else:
                jam_timer = 0  # Reset if intake moves normally
            wait(5, MSEC)

    while True:
        anti_jam()
        wait(10, MSEC)  # Reduce unnecessary CPU usage'''
        



