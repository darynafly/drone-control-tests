import keyboard
import struct
import time

import socket
import struct

### Connection
# Connect to a Betaflight drone via TCP
host = '192.168.1.6'  # IP address of the drone
port = 5761  # port typically used for TCP-based connections with Betaflight

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

MSP_SET_MOTOR = 214

# Define the base throttle value and increment for adjustments
BASE_THROTTLE = 1300  # Base throttle when space is pressed
ADJUSTMENT = 10      # Adjustment value for directional controls

# Initialize motor values
motor1 = 0
motor2 = 0
motor3 = 0
motor4 = 0

# Keep track of previous key states to detect single presses
key_states = {
    "space": False,
    "left": False,
    "right": False,
    "up": False,
    "down": False,
    "a": False,  # for global adjustment
    "s": False,  # for global stop all motors
}


### Main functions
def construct_msp_message(command: int, data: list = []) -> bytearray:
    """Creates expected formatting.

    Command frame format:
    Header: `$M<`
    Body: length of `data` + `command` + `data` values
    Tail: checksum

    Args:
        command (int): MSP command
        data (list, optional): list of params (length is varying). Defaults to [].

    Returns:
        bytearray: command frame
    """
    message = bytearray()
    # header
    message.append(ord('$'))
    message.append(ord('M'))
    message.append(ord('<'))
    # body
    message.append(len(data))
    message.append(command)
    checksum = len(data) ^ command
    for byte in data:
        message.append(byte)
        checksum ^= byte
    # tail
    message.append(checksum)
    return message

def send_msp_message(command: int, data: list = []):
    """Send prepared command via serial

    Args:
        command (int): MSP command
        data (list, optional): list of params. Defaults to [].
    """
    message = construct_msp_message(command, data)
    sock.sendall(message)

def set_motor_values(motor1: int, motor2: int, motor3: int, motor4: int):
    """Set throttle per motor
    
    Args:
        motor1 (int): motor 1 throttle
        motor2 (int): motor 2 throttle
        motor3 (int): motor 3 throttle
        motor4 (int): motor 4 throttle
    """
    print(motor1, motor2, motor3, motor4)
    motor_values = [motor1, motor2, motor3, motor4, 0, 0, 0, 0]
    send_msp_message(MSP_SET_MOTOR, struct.pack('<8H', *motor_values))

# Function to update motors based on inputs
def update_motors():
    global motor1, motor2, motor3, motor4

    # Base throttle is set when space is pressed, else zero
    if keyboard.is_pressed("space"):
        if not key_states["space"]:  # Check for first press
            motor1 = BASE_THROTTLE
            motor2 = BASE_THROTTLE
            motor3 = BASE_THROTTLE + 100
            motor4 = BASE_THROTTLE
            key_states["space"] = True
    #else:
    #    motor1 = motor2 = motor3 = motor4 = 0
    #    key_states["space"] = False

    if True:
        #key_states["space"]:
        # Increment motor values based on single presses of arrow keys
        if keyboard.is_pressed("left") and not key_states["left"]:
            motor1 += ADJUSTMENT
            motor2 += ADJUSTMENT
            key_states["left"] = True
        elif not keyboard.is_pressed("left"):
            key_states["left"] = False

        if keyboard.is_pressed("right") and not key_states["right"]:
            motor3 += ADJUSTMENT
            motor4 += ADJUSTMENT
            key_states["right"] = True
        elif not keyboard.is_pressed("right"):
            key_states["right"] = False

        if keyboard.is_pressed("up") and not key_states["up"]:
            motor1 += ADJUSTMENT
            motor3 += ADJUSTMENT
            key_states["up"] = True
        elif not keyboard.is_pressed("up"):
            key_states["up"] = False

        if keyboard.is_pressed("down") and not key_states["down"]:
            motor2 += ADJUSTMENT
            motor4 += ADJUSTMENT
            key_states["down"] = True
        elif not keyboard.is_pressed("down"):
            key_states["down"] = False

        # Global increase for all motors when "A" is pressed
        if keyboard.is_pressed("a") and not key_states["a"]:
            motor1 += ADJUSTMENT
            motor2 += ADJUSTMENT
            motor3 += ADJUSTMENT
            motor4 += ADJUSTMENT
            key_states["a"] = True
        elif not keyboard.is_pressed("a"):
            key_states["a"] = False

        # Global increase for all motors when "A" is pressed
        if keyboard.is_pressed("d") and not key_states["d"]:
            motor1 -= ADJUSTMENT
            motor2 -= ADJUSTMENT
            motor3 -= ADJUSTMENT
            motor4 -= ADJUSTMENT
            key_states["d"] = True
        elif not keyboard.is_pressed("d"):
            key_states["d"] = False

    # Stop
    if keyboard.is_pressed("s") and not key_states["s"]:
        motor1 = 0
        motor2 = 0
        motor3 = 0
        motor4 = 0
        key_states["s"] = True
        key_states["space"] = False
        # Set the motor values
        set_motor_values(motor1, motor2, motor3, motor4)
        time.sleep(0.5)
    elif not keyboard.is_pressed("s"):
        key_states["s"] = False

    # Set the motor values
    set_motor_values(motor1, motor2, motor3, motor4)

# Main loop to continuously check for keyboard inputs
try:
    while True:
        update_motors()
        time.sleep(0.1)  # Adjust this delay as needed
except KeyboardInterrupt:
    print("Program stopped.")
    set_motor_values(0, 0, 0, 0)
    time.sleep(0.5)
    sock.close()
