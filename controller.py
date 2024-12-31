import pygame

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Check for connected joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
else:
    joystick = pygame.joystick.Joystick(0)  # Use the first connected joystick
    joystick.init()
    print(f"Joystick connected: {joystick.get_name()}")

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed.")
        elif event.type == pygame.JOYAXISMOTION:
            print(f"Axis {event.axis} moved to {event.value}.")
        elif event.type == pygame.JOYHATMOTION:
            print(f"Hat {event.hat} moved to {event.value}.")

pygame.quit()
