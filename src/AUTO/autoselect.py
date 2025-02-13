import time

# Autonomous Mode Options (Formatted for VEX Controller)
auto_modes = [
    "[NO AUTO]",
    "[RED LEFT RING]",
    "[BLUE RIGHT RING]",
    "[RED RIGHT STAKE]",
    "[BLUE LEFT STAKE]"
]

AutoSelect = 0  # Default to NO AUTO

def update_auto_display():
    """ Updates the controller screen with a simple animation effect """
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    
    # Display Auto Mode Name
    controller_1.screen.print(auto_modes[AutoSelect])

    # Show Navigation Controls
    controller_1.screen.set_cursor(2, 1)
    controller_1.screen.print("<< SELECT >>")

    # Rumble Feedback
    controller_1.rumble(".")

def fancy_scroll_effect():
    """ Creates a quick flashing effect when switching modes """
    for _ in range(2):
        controller_1.screen.clear_screen()
        wait(50, MSEC)
        update_auto_display()

def onevent_controller_1buttonL1_pressed_0():
    """ Scroll to the NEXT autonomous mode """
    global AutoSelect
    AutoSelect = (AutoSelect + 1) % len(auto_modes)
    controller_1.rumble(".-")  
    fancy_scroll_effect()

def onevent_controller_1buttonL2_pressed_0():
    """ Scroll to the PREVIOUS autonomous mode """
    global AutoSelect
    AutoSelect = (AutoSelect - 1) % len(auto_modes)
    controller_1.rumble("-..")  
    fancy_scroll_effect()

def when_started5():
    """ Initialize the auto selector with a clean display """
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("AUTO SELECT MODE")

    # Quick Flashing Effect
    for _ in range(3):
        controller_1.screen.print(".")
        wait(200, MSEC)

    update_auto_display()  # Show first selection
