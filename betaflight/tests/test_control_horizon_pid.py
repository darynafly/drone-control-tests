import struct
import time
import socket
import math

# Connection settings
host = '192.168.1.253'
port = 5761

# MSP Commands
MSP_RAW_IMU = 102
MSP_SET_MOTOR = 214

# Base throttle and adjustments
BASE_THROTTLE = 1250
MAX_MOTOR_VALUE = 1450
MIN_MOTOR_VALUE = 1000
THRESHOLD = 5  # Sensitivity threshold in degrees

ADJUSTMENT_PITCH = 0.1
ADJUSTMENT_ROLL = 0.1

# Target pitch and roll in degrees for stable hover
TARGET_PITCH = 0.0
TARGET_ROLL = -0.5

# PID gains
P_GAIN = 0.15  # Reduced for smoother control
I_GAIN = 0.008  # Reduced integral gain to limit drift
D_GAIN = 0.03  # Reduced to avoid sharp responses

# Initialize PID controllers
pitch_integral = 0
roll_integral = 0
prev_pitch_error = 0
prev_roll_error = 0

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

def calculate_pid_correction(imu_data):
    """Calculate motor corrections using a PID controller for target pitch and roll."""
    global pitch_integral, roll_integral, prev_pitch_error, prev_roll_error
    
    ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
    pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
    roll = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

    print("Current Pitch:", pitch, "Current Roll:", roll)

    pitch_error = TARGET_PITCH - pitch
    roll_error = TARGET_ROLL - roll

    # Proportional, Integral, and Derivative calculations for pitch
    pitch_integral += pitch_error
    pitch_derivative = pitch_error - prev_pitch_error
    prev_pitch_error = pitch_error
    pitch_adjust = (P_GAIN * pitch_error) + (I_GAIN * pitch_integral) + (D_GAIN * pitch_derivative)

    # Proportional, Integral, and Derivative calculations for roll
    roll_integral += roll_error
    roll_derivative = roll_error - prev_roll_error
    prev_roll_error = roll_error
    roll_adjust = (P_GAIN * roll_error) + (I_GAIN * roll_integral) + (D_GAIN * roll_derivative)

    # Motor adjustments based on pitch and roll adjustments
    motor1_adjust = pitch_adjust + roll_adjust
    motor2_adjust = pitch_adjust - roll_adjust
    motor3_adjust = -pitch_adjust - roll_adjust
    motor4_adjust = -pitch_adjust + roll_adjust

    return motor1_adjust, motor2_adjust, motor3_adjust, motor4_adjust

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

def gradual_takeoff():
    """Gradually increase motor throttle for a smooth takeoff."""
    global motor1, motor2, motor3, motor4
    takeoff_speed = BASE_THROTTLE
    while takeoff_speed < BASE_THROTTLE + 250:
        motor1 = motor2 = motor3 = motor4 = takeoff_speed
        imu_data = get_raw_imu()
        if imu_data:
            motor1_adjust, motor2_adjust, motor3_adjust, motor4_adjust = calculate_tilt_correction(imu_data)
            motor1 += motor1_adjust
            motor2 += motor2_adjust + 30
            motor3 += motor3_adjust + 30
            motor4 += motor4_adjust + 30
            print(takeoff_speed)
            set_motor_values(motor1, motor2, motor3, motor4)
            takeoff_speed += 5
            time.sleep(0.01)
    print("takeoff")
    time.sleep(0.2)
    while takeoff_speed > 1000:
        motor1 = motor2 = motor3 = motor4 = takeoff_speed
        imu_data = get_raw_imu()
        if imu_data:
            motor1_adjust, motor2_adjust, motor3_adjust, motor4_adjust = calculate_tilt_correction(imu_data)
            motor1 += motor1_adjust
            motor2 += motor2_adjust
            motor3 += motor3_adjust
            motor4 += motor4_adjust
            set_motor_values(motor1, motor2, motor3, motor4)
            takeoff_speed -= 10
            print(takeoff_speed)
            time.sleep(0.01)
    print("down")

def hover_one_point():
    """Attempt to hold drone stable at one point."""
    global motor1, motor2, motor3, motor4
    motor1 = motor2 = motor3 = motor4 = BASE_THROTTLE
    set_motor_values(motor1, motor2, motor3, motor4)
    time.sleep(0.1)

# Main control loop
try:
    gradual_takeoff()  # Begin with a gentle takeoff
    #start_time = time.time()
    #while time.time() - start_time < 4:  # Run for a few seconds
    #    imu_data = get_raw_imu()
    #    if imu_data:
    #        motor1_adjust, motor2_adjust, motor3_adjust, motor4_adjust = calculate_pid_correction(imu_data)
    #        motor1 += motor1_adjust
    #        motor2 += motor2_adjust + 50
    #        motor3 += motor3_adjust
    #        motor4 += motor4_adjust + 50
    #        set_motor_values(motor1, motor2, motor3, motor4)
    #    time.sleep(0.01)  # Control loop frequency

    # Stop motors after test
    set_motor_values(0, 0, 0, 0)
except KeyboardInterrupt:
    print("Program interrupted. Stopping motors...")
    set_motor_values(0, 0, 0, 0)
finally:
    sock.close()
