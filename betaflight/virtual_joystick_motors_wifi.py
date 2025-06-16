import pygame
import sys
import socket
import struct
from time import sleep

# Initialize pygame
pygame.init()

# Screen settings
WIDTH, HEIGHT = 600, 400
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Motor Control Simulator")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Joystick and motor settings
JOYSTICK_RADIUS = 45
JOYSTICK_SPEED = 2
MAX_MOTOR_SPEED = 1500  # Max speed for each motor
BASE_MOTOR_SPEED = 1000  # Starting speed
GENERAL_SPEED = 1000 # General speed for hover

# Starting positions for joysticks
left_joystick_pos = [150, 200]
right_joystick_pos = [450, 200]

# Initial motor speeds
motor_speeds = [BASE_MOTOR_SPEED] * 4

### Connection to the drone via TCP
host = '10.0.0.1'  # IP address of the drone
#host = '192.168.1.11' 
port = 5761            # Port for TCP-based connections with Betaflight
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

### MSP commands
MSP_SET_MOTOR = 214

def construct_msp_message(command: int, data: list = []) -> bytearray:
    """Creates expected formatting for MSP message."""
    message = bytearray()
    message.extend([ord('$'), ord('M'), ord('<'), len(data), command])
    checksum = len(data) ^ command
    for byte in data:
        message.append(byte)
        checksum ^= byte
    message.append(checksum)
    return message

def send_msp_message(command: int, data: list = []):
    """Send prepared command via socket connection."""
    message = construct_msp_message(command, data)
    sock.sendall(message)

def set_motor_values(motor1: int, motor2: int, motor3: int, motor4: int):
    """Send throttle values to each motor."""
    motor_values = [motor1, motor2, motor3, motor4, 0, 0, 0, 0]
    send_msp_message(MSP_SET_MOTOR, struct.pack('<8H', *motor_values))

def update_motor_speeds():
    """Calculate motor speeds based on joystick positions."""
    increment = 400
    # Throttle and pitch (left joystick)
    throttle_adjust = ((200 - left_joystick_pos[1]) / 200) * increment # Vertical movement on left joystick
    GENERAL_SPEED = BASE_MOTOR_SPEED + throttle_adjust
    
    pitch_adjust = ((left_joystick_pos[0] - 150) / 150) * increment    # Horizontal movement on left joystick

    # Roll and yaw (right joystick)
    roll_adjust = ((right_joystick_pos[0] - 450) / 150) * increment    # Horizontal movement on right joystick
    yaw_adjust = ((200 - right_joystick_pos[1]) / 200) * increment       # Vertical movement on right joystick

    throttle_adjust = int(throttle_adjust)
    pitch_adjust = int(pitch_adjust)
    roll_adjust = int(roll_adjust)
    yaw_adjust = int(yaw_adjust)

    # Apply adjustments to motor speeds
    motor_speeds[0] = BASE_MOTOR_SPEED + throttle_adjust + pitch_adjust - roll_adjust - yaw_adjust  # Front-left
    motor_speeds[1] = BASE_MOTOR_SPEED + throttle_adjust - pitch_adjust + roll_adjust + yaw_adjust  # Front-right
    motor_speeds[2] = BASE_MOTOR_SPEED + throttle_adjust - pitch_adjust - roll_adjust - yaw_adjust  # Rear-left
    motor_speeds[3] = BASE_MOTOR_SPEED + throttle_adjust + pitch_adjust + roll_adjust + yaw_adjust  # Rear-right

    # Clamp motor speeds to 0 and MAX_MOTOR_SPEED
    for i in range(4):
        motor_speeds[i] = int(max(GENERAL_SPEED, min(MAX_MOTOR_SPEED, motor_speeds[i])))

    # Send motor values to the drone
    set_motor_values(*motor_speeds)

# Main loop
running = True
while running:
    # Fill the screen
    screen.fill(WHITE)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sock.close()
            sys.exit()

    # Key handling
    keys = pygame.key.get_pressed()
    
    # Left joystick (throttle and pitch) controls
    if keys[pygame.K_w]:  # Throttle up
        left_joystick_pos[1] -= JOYSTICK_SPEED
    if keys[pygame.K_s]:  # Throttle down
        left_joystick_pos[1] += JOYSTICK_SPEED
    if keys[pygame.K_a]:  # Pitch left
        left_joystick_pos[0] -= JOYSTICK_SPEED
    if keys[pygame.K_d]:  # Pitch right
        left_joystick_pos[0] += JOYSTICK_SPEED

    # Right joystick (roll and yaw) controls
    if keys[pygame.K_UP]:    # Yaw up
        right_joystick_pos[1] -= JOYSTICK_SPEED
    if keys[pygame.K_DOWN]:  # Yaw down
        right_joystick_pos[1] += JOYSTICK_SPEED
    if keys[pygame.K_LEFT]:  # Roll left
        right_joystick_pos[0] -= JOYSTICK_SPEED
    if keys[pygame.K_RIGHT]: # Roll right
        right_joystick_pos[0] += JOYSTICK_SPEED

    # Update motor speeds based on joystick positions
    update_motor_speeds()
    print(f"Motor Speeds: {motor_speeds}")

    # Draw left joystick
    pygame.draw.circle(screen, RED, left_joystick_pos, JOYSTICK_RADIUS)
    pygame.draw.circle(screen, BLACK, (150, 200), JOYSTICK_RADIUS + 10, 2)  # Boundary for left joystick

    # Draw right joystick
    pygame.draw.circle(screen, BLUE, right_joystick_pos, JOYSTICK_RADIUS)
    pygame.draw.circle(screen, BLACK, (450, 200), JOYSTICK_RADIUS + 10, 2)  # Boundary for right joystick

    # Update the display
    pygame.display.flip()
    
    # Cap the frame rate
    pygame.time.Clock().tick(60)
