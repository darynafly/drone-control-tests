import struct
import time
import socket
import math

# Connection settings
host = '192.168.1.254'
#host = '10.0.0.1'
port = 5761

# MSP Commands
MSP_RAW_IMU = 102
MSP_SET_MOTOR = 214

# Base throttle and adjustments
BASE_THROTTLE = 1300
ADJUSTMENT = 10
TARGET_PITCH = 0.6  # Target pitch in degrees
TARGET_ROLL = 3.26   # Target roll in degrees
MAX_MOTOR_VALUE = 1500  # Max motor value
MIN_MOTOR_VALUE = 0     # Min motor value

# Initialize motor values
motor1 = motor2 = motor3 = motor4 = 0

# Connect to the drone
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

def construct_msp_message(command: int, data: list = []) -> bytearray:
    message = bytearray([ord('$'), ord('M'), ord('<'), len(data), command])
    checksum = len(data) ^ command
    for byte in data:
        message.append(byte)
        checksum ^= byte
    message.append(checksum)
    return message

def send_msp_message(command: int, data: list = []):
    message = construct_msp_message(command, data)
    sock.sendall(message)

def read_msp_response() -> tuple:
    """Read data from the drone and check integrity."""
    header = sock.recv(3)
    if header != b'$M>':
        return None
    data_length = sock.recv(1)[0]
    code = sock.recv(1)[0]
    data = sock.recv(data_length)
    checksum = sock.recv(1)[0]
    calculated_checksum = data_length ^ code
    for byte in data:
        calculated_checksum ^= byte
    if calculated_checksum != checksum:
        return None
    return (code, data)

def get_raw_imu() -> tuple:
    """Read raw IMU data and decode it."""
    send_msp_message(MSP_RAW_IMU)
    time.sleep(0.005)
    response = read_msp_response()
    if response and response[0] == MSP_RAW_IMU:
        imu = struct.unpack(f'<{len(response[1]) // 2}h', response[1])  # Decode as signed
        return imu
    return None

def calculate_tilt_correction(imu_data):
    """Calculate motor corrections to maintain target pitch and roll."""
    ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
    pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    roll = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

    print("Current Pitch:", pitch, "Current Roll:", roll)
    
    # Calculate deviation from target pitch and roll
    pitch_error = pitch - TARGET_PITCH
    roll_error = roll - TARGET_ROLL

    # Adjust motors based on pitch and roll errors
    correction = [0, 0, 0, 0]
    pitch_adjustment = int(pitch_error * ADJUSTMENT)
    roll_adjustment = int(roll_error * ADJUSTMENT)

    correction[0] -= pitch_adjustment  # Adjust front motors
    correction[2] += pitch_adjustment  # Adjust rear motors
    correction[1] -= roll_adjustment   # Adjust left motors
    correction[3] += roll_adjustment   # Adjust right motors

    return correction

def apply_bounds(value):
    """Ensure motor values remain within valid limits."""
    return max(MIN_MOTOR_VALUE, min(MAX_MOTOR_VALUE, value))

def set_motor_values(motor1: int, motor2: int, motor3: int, motor4: int):
    """Set throttle per motor, applying bounds to each."""
    motor_values = [
        motor1, motor2,
        motor3, motor4,
        0, 0, 0, 0
    ]
    print("Motor Values:", motor_values)
    send_msp_message(MSP_SET_MOTOR, struct.pack('<8H', *motor_values))


# Main control loop
try:
    while True:
        set_motor_values(0, 0, 0, 0)
        time.sleep(0.01)  # Control loop frequency
except KeyboardInterrupt:
    print("Program interrupted. Stopping motors...")
    set_motor_values(0, 0, 0, 0)
    sock.close()
