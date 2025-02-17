

def draw_speedometer():
    brain.screen.clear_screen()

    # Draw speedometer outline
    brain.screen.draw_circle(160, 120, 50)  # Outer circle
    brain.screen.print(140, 170, "Speed")

    # Get motor speed (0-100%)
    speed = RightMotors.velocity(PERCENT)
    
    # Convert speed to angle (-90° to 90°)
    angle = -90 + (speed * 1.8)

    # Calculate needle position
    needle_x = 160 + 40 * math.cos(angle)
    needle_y = 120 + 40 * math.sin(angle)

    # Draw speedometer needle
    brain.screen.draw_line(160, 120, needle_x, needle_y)

    # Render updated screen
    brain.screen.render()

