import pygame
import time
import sys

# Pygame setup
pygame.init()
pygame.joystick.init()

# Check for joystick
if pygame.joystick.get_count() == 0:
    print("No joystick detected! Please connect a controller.")
    sys.exit(1)

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Initialized joystick: {joystick.get_name()}")
print("Press and hold the L2 (Left Trigger) to identify its axis.")
print("Press Ctrl+C to quit.")

num_axes = joystick.get_numaxes()
print(f"Found {num_axes} axes. Now monitoring values...")

try:
    while True:
        pygame.event.pump() # Update joystick states
        
        output = []
        for i in range(num_axes):
            axis_value = joystick.get_axis(i)
            # Format nicely
            output.append(f"Axis {i}: {axis_value: 6.2f}")
        
        # Print all axis values on one line
        print(" | ".join(output), end='\r')
        
        time.sleep(0.05) # Refresh rate

except KeyboardInterrupt:
    print("\nDone.")
    pygame.quit()
    sys.exit(0)
