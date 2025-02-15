

def Move_In_direction_Degree_Speed(Move_In_direction_Degree_Speed__Degree, Move_In_direction_Degree_Speed__Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # LINKED WITH CURRENT PID FOR MOVEMENT VELOCITY
    if Move_In_direction_Degree_Speed__Degree > 0:
        RightMotors.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        Right_front.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        LeftMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Left_Front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
    else:
        RightMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Right_front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        LeftMotors.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        Left_Front.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)


def Forward_PID_Distance_Max_Speed(Forward_PID_Distance_Max_Speed__Distance, Forward_PID_Distance_Max_Speed__Max_Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CURRENT PID - DISTANCE IN INCHES
    # PID VALUES: 0.5 - 0.01 - 0.1 - 0.05
    error = 0
    Kp = 0.45
    Ki = 0
    Kd = 0.01
    Dellay = 0.05

    Distance_travled = 0
    Inertial21.set_rotation(0, DEGREES)
    RightMotors.set_velocity(5, PERCENT)
    Right_front.set_velocity(5, PERCENT)
    LeftMotors.set_velocity(5, PERCENT)
    Left_Front.set_velocity(5, PERCENT)
    RightMotors.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)
    LeftMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)
    while True:
        imput = Inertial21.rotation(DEGREES)
        error = 0 - imput
        Proportional = error
        Distance_travled = ((math.fabs(Right_front.position(DEGREES)) / 360) * (2.75 * pi) + ((math.fabs(Left_Front.position(DEGREES)) / 360) * (2.75 * pi) + ((math.fabs(RightMotors.position(DEGREES)) / 360) * (2.75 * pi) + (math.fabs(LeftMotors.position(DEGREES)) / 360) * (2.75 * pi)))) / 4
        integral = (integral + error) * Dellay
        derivitive = (error - Previus_error) * Dellay
        direction = (Kp * Proportional + (Ki * integral + Kd * derivitive))
        Previus_error = error
        Move_In_direction_Degree_Speed((direction * Forward_PID_Distance_Max_Speed__Max_Speed) / 33, Forward_PID_Distance_Max_Speed__Max_Speed)
        RightMotors.spin(REVERSE)
        Right_front.spin(REVERSE)
        LeftMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        if Forward_PID_Distance_Max_Speed__Distance < Distance_travled:
            RightMotors.stop()
            Right_front.stop()
            LeftMotors.stop()
            Left_Front.stop()
            break
        wait(5, MSEC)





        


# create a function for handling the starting and stopping of all autonomous tasks


