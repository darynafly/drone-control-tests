import socket
import struct
from time import sleep

# Connection
host = '192.168.1.254'  # IP address of the drone
port = 5761  # Port typically used for TCP-based connections with Betaflight
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

# MSP Commands
MSP_RAW_IMU = 102
MSP_SET_MOTOR = 214

# Proportional control gain
kp_roll = 1.0
kp_pitch = 1.0
BASE_SPEED = 1350  # Base throttle value

def construct_msp_message(command: int, data: list = []) -> bytearray:
    message = bytearray()
    message.extend([ord('$'), ord('M'), ord('<')])
    message.append(len(data))
    message.append(command)
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
    send_msp_message(MSP_RAW_IMU)
    sleep(0.01)
    response = read_msp_response()
    if response and response[0] == MSP_RAW_IMU:
        imu = struct.unpack('<9h', response[1])
        accel_x, accel_y, accel_z = imu[0], imu[1], imu[2]
        gyro_x, gyro_y, gyro_z = imu[3], imu[4], imu[5]
        return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
    return None

def set_motor_values(motor1: int, motor2: int, motor3: int, motor4: int):
    motor_values = [motor1, motor2, motor3, motor4, 0, 0, 0, 0]
    send_msp_message(MSP_SET_MOTOR, struct.pack('<8H', *motor_values))

def stabilize_horizon():
    while True:
        imu_data = get_raw_imu()
        if imu_data:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data

            # Error calculations
            roll_error = -accel_x  # Negative to oppose the tilt
            pitch_error = -accel_y

            # Adjustments based on proportional control
            roll_adjust = int(kp_roll * roll_error)
            pitch_adjust = int(kp_pitch * pitch_error)

            # Motor values
            motor1 = BASE_SPEED + pitch_adjust + roll_adjust
            motor2 = BASE_SPEED + pitch_adjust - roll_adjust
            motor3 = BASE_SPEED - pitch_adjust + roll_adjust
            motor4 = BASE_SPEED - pitch_adjust - roll_adjust

            # Set motor values
            set_motor_values(motor1, motor2, motor3, motor4)

            # Short sleep to allow for IMU update
            sleep(0.05)

try:
    stabilize_horizon()
except KeyboardInterrupt:
    # Stop all motors on exit
    set_motor_values(0, 0, 0, 0)
    sock.close()
