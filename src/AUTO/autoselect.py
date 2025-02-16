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
    """Updates the controller screen with a dynamic animation effect."""
    controller_1.screen.clear_screen()

    # Display selected Autonomous Mode with background color (if supported)
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("> {auto_modes[AutoSelect]} <")

    # Display Navigation Instructions with bold styling
    controller_1.screen.set_cursor(2, 1)
  
    controller_1.screen.print("<< SELECT >>")


    # Rumble feedback with more intensity for better interaction
    controller_1.rumble("...")

def fancy_scroll_effect():
    """Creates a cool flashing effect when switching modes."""
    for _ in range(2):
        controller_1.screen.clear_screen()
        wait(50, MSEC)
        update_auto_display()

def smooth_scroll_effect():
    """Smooth scrolling text effect when switching modes."""
    for i in range(len(auto_modes[AutoSelect])):
        controller_1.screen.set_cursor(1, 1)
        controller_1.screen.print(auto_modes[AutoSelect][:i + 1])
        wait(50, MSEC)
    update_auto_display()

def onevent_controller_1buttonL1_pressed_0():
    """Scroll to the NEXT autonomous mode."""
    global AutoSelect
    AutoSelect = (AutoSelect + 1) % len(auto_modes)
    controller_1.rumble("-.-")  # Short rumble for next mode
    smooth_scroll_effect()  # Apply smooth scrolling effect

def onevent_controller_1buttonL2_pressed_0():
    """Scroll to the PREVIOUS autonomous mode."""
    global AutoSelect
    AutoSelect = (AutoSelect - 1) % len(auto_modes)
    controller_1.rumble("-..")  # Short rumble for previous mode
    smooth_scroll_effect()  # Apply smooth scrolling effect

def when_started5():
    """Initialize the auto selector with a visually clean display."""
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("AUTO SELECT MODE")

    # Quick flashing effect to draw attention
    for _ in range(3):
        controller_1.screen.print(".")
        wait(200, MSEC)

    update_auto_display()  # Show initial auto mode'''
