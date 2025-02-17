




#region VEXcode Generated Robot Configuration
from vex import *
'''from DRIVER_FUNCTIONS.drive import ondriver_drivercontrol_0, ondriver_drivercontrol_1, ondriver_drivercontrol_2, ondriver_drivercontrol_3, onauton_autonomous_0, onevent_controller_1axis2Changed_0, onevent_controller_1axis3Changed_0
from AUTO.autoselect import onevent_controller_1buttonL1_pressed_0
from INIT.init import when_started4, onevent_stop_initialize_0
from DRIVER_FUNCTIONS.LB import ondriver_drivercontrol_4
from AUTO.autonomous import onauton_autonomous_0
from OTHER.testcode import when_started2, when_started3
from AUTO.autoselect import when_started5'''
import urandom # type: ignore

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
RightMotors_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
RightMotors_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
RightMotors = MotorGroup(RightMotors_motor_a, RightMotors_motor_b)
LeftMotors_motor_a = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
LeftMotors_motor_b = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
LeftMotors = MotorGroup(LeftMotors_motor_a, LeftMotors_motor_b)
Right_front = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
Left_Front = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
digital_out_b = DigitalOut(brain.three_wire_port.b)
rotation_15 = Rotation(Ports.PORT15, False)
Lady_Brown_motor_a = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
Lady_Brown_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
Lady_Brown = MotorGroup(Lady_Brown_motor_a, Lady_Brown_motor_b)
Inertial21 = Inertial(Ports.PORT5)
digital_out_g = DigitalOut(brain.three_wire_port.g)
optical_4 = Optical(Ports.PORT4)
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
digital_out_e = DigitalOut(brain.three_wire_port.e)
gps_3 = Gps(Ports.PORT3, 0.00, 5.00, MM, -90)
distance_11 = Distance(Ports.PORT11)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


"""# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()"""


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

remote_control_code_enabled = True
vexcode_brain_precision = 0
vexcode_console_precision = 0
vexcode_controller_1_precision = 0
message1 = Event()
forward_move = Event()
Back_move = Event()
Stop = Event()
turn_right = Event()
'''turn = Event()'''
calibrate = Event()
stop_initialize = Event()
Auto_Stop = Event()
turn_left = Event()
start_auto = Event()
intake_forward = Event()
intake_backward = Event()
DOon = False
INTAKEF = False
INTAKER = False
LB = False
DOon2 = False
Blue = False
Red = False
Intake_Control = False
Intake_running = False
myVariable = 0
volocity = 0
Right_Axis = 0
Left_Axis = 0
IntakeStake = 0
Degree = 0
pi = 0
movement = 0
distance1 = 0
time1 = 0
rot = 0
turn1 = 0
LadyBrown_Up = 0
LadyBrown_score = 0
LadyBrown = 0
Right_turn = 0
Left_turn = 0
DriveState = 0
start = 0
Next = 0
dos = 0
tog = 0
error = 0
output = 0
Kp = 0
Ki = 0
Kd = 0
Dellay = 0
Distance_travled = 0
imput = 0
Proportional = 0
integral = 0
derivitive = 0
direction = 0
Previus_error = 0
AutoSelect = 0
X_Start = 0
Y_Start = 0
Y_End = 0
X_End = 0
Angle = 0
Distnce2 = 0
Distance2 = 0
Turn_Angle = 0


