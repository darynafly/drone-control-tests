import struct
import time
import socket
import math

# Connection settings
host = '192.168.1.251'
#host = '10.0.0.1'
port = 5761

# MSP Commands
MSP_RAW_IMU = 102
MSP_SET_MOTOR = 214

# Base throttle and adjustments
BASE_THROTTLE = 1400
ADJUSTMENT_PITCH = 0.1
ADJUSTMENT_ROLL = 0.1
TARGET_PITCH = 0.4  # Target pitch in degrees
TARGET_ROLL = 1 # Target roll in degrees
MAX_MOTOR_VALUE = 1450  # Max motor value
MIN_MOTOR_VALUE = 1000  # Min motor value
THRESHOLD = 5  # Accuracy threshold in degrees

# Initialize motor values
motor1 = motor2 = motor3 = motor4 = BASE_THROTTLE

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
    """Calculate motor corrections to maintain target pitch and roll with accuracy threshold."""
    ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
    pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    roll = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

    print("Current Pitch:", pitch, "Current Roll:", roll)
    
    # Calculate deviation from target pitch and roll
    pitch_error = TARGET_PITCH - pitch
    roll_error = TARGET_ROLL - roll

    # Only apply correction if the error exceeds the threshold
    motor1_adjust = motor2_adjust = motor3_adjust = motor4_adjust = 0
    if abs(pitch_error) > THRESHOLD:
        motor1_adjust += pitch_error * ADJUSTMENT_PITCH
        motor2_adjust += pitch_error * ADJUSTMENT_PITCH
        motor3_adjust -= pitch_error * ADJUSTMENT_PITCH
        motor4_adjust -= pitch_error * ADJUSTMENT_PITCH

    if abs(roll_error) > THRESHOLD:
        motor1_adjust += roll_error * ADJUSTMENT_ROLL
        motor4_adjust += roll_error * ADJUSTMENT_ROLL
        motor2_adjust -= roll_error * ADJUSTMENT_ROLL
        motor3_adjust -= roll_error * ADJUSTMENT_ROLL

    return motor1_adjust, motor2_adjust, motor3_adjust, motor4_adjust

def apply_bounds(value):
    """Ensure motor values remain within valid limits."""
    return max(MIN_MOTOR_VALUE, min(MAX_MOTOR_VALUE, int(value)))

def set_motor_values(motor1: int, motor2: int, motor3: int, motor4: int):
    """Set throttle per motor, applying bounds to each."""
    motor_values = [
        apply_bounds(motor1), apply_bounds(motor2),
        apply_bounds(motor3), apply_bounds(motor4),
        0, 0, 0, 0
    ]
    print("Motor Values:", motor_values)
    send_msp_message(MSP_SET_MOTOR, struct.pack('<8H', *motor_values))

def takeoff():
    global motor1, motor2, motor3, motor4
    motor4 = BASE_THROTTLE + 25
    motor2 = BASE_THROTTLE + 50
    motor3 = BASE_THROTTLE
    motor1 = BASE_THROTTLE + 20
    set_motor_values(motor1, motor2, motor3, motor4)
    time.sleep(0.1)

# Main control loop
try:
    takeoff()
    start_time = time.time()
    while time.time() - start_time < 3:  # Run for seconds
        imu_data = get_raw_imu()
        if imu_data:
            motor1_adjust, motor2_adjust, motor3_adjust, motor4_adjust = calculate_tilt_correction(imu_data)
            motor1 += motor1_adjust
            motor2 += motor2_adjust
            motor3 += motor3_adjust
            motor4 += motor4_adjust
            set_motor_values(motor1, motor2, motor3, motor4)
        time.sleep(0.01)  # Control loop frequency

    # Stop motors after
    while True:
        set_motor_values(0, 0, 0, 0)
except KeyboardInterrupt:
    print("Program interrupted. Stopping motors...")
    set_motor_values(0, 0, 0, 0)
finally:
    sock.close()
