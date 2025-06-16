import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# Constants
g = 9.81  # Gravitational acceleration in m/s²
dt = 0.001  # Time step in seconds (adjust based on your loop timing)
x_accel_threshold = 400.0  # Threshold for acceleration in millig
y_accel_threshold = 400.0  # Threshold for acceleration in millig
small_value_adjustment = 0.1  # Small value to adjust positions

# Initial conditions
position_x = 0.0  # Initial position in X
position_y = 0.0  # Initial position in Y
position_z = 0.0  # Initial position in Z

import socket
import struct
import time

### Connection
# Connect to a Betaflight drone via TCP
host = '192.168.1.2'  # IP address of the drone
port = 5761  # port typically used for TCP-based connections with Betaflight

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

### MSP commands
# out messages usually start with 1 (1**)
MSP_STATUS = 101
MSP_RAW_IMU = 102
# in messages usually start with 2 (2**)
MSP_SET_MOTOR = 214

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

def read_msp_response() -> tuple:
    """Read data

    Returns:
        tuple: command code and its values
    """
    header = sock.recv(3)  # read header
    if header != b'$M>':
        return None
    # read body
    data_length = sock.recv(1)[0]
    code = sock.recv(1)[0]
    data = sock.recv(data_length)
    # read tail
    checksum = sock.recv(1)[0]
    # check if the command is of the expected length
    calculated_checksum = data_length ^ code
    for byte in data:
        calculated_checksum ^= byte
    if calculated_checksum != checksum:
        return None

    return (code, data)

def get_raw_imu() -> tuple:
    """Read raw IMU data and decode it as signed 16-bit integers.

    Returns:
        tuple | None: tuple - processed data, None - received data doesn't correspond to sent command
    """
    send_msp_message(MSP_RAW_IMU)  # send request
    time.sleep(0.009)  # wait for reply
    response = read_msp_response()  # get reply

    if response and response[0] == MSP_RAW_IMU:
        # Decode as signed 16-bit integers
        imu = struct.unpack('<9h', response[1])  # AccX, AccY, AccZ, GyroX, GyroY, GyroZ
        #imu = struct.unpack(f'<{len(response[1]) // 2}H', response[1])  # decode
        return imu
    return None

# Set up plotting
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()
ax.set_title('Acceleration X over Time')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Acceleration (milli-g)')
ax.set_ylim(-5000, 5000)  # Set y-axis limits based on expected values

# Initialize deque for storing acceleration values for plotting
acc_x_values = deque(maxlen=100)  # Store last 100 acceleration values
time_values = deque(maxlen=100)    # Store corresponding time values
start_time = time.time()            # Record the start time

def get_raw_imu() -> tuple:
    """Read raw IMU data and decode it as signed 16-bit integers.

    Returns:
        tuple | None: tuple - processed data, None - received data doesn't correspond to sent command
    """
    send_msp_message(MSP_RAW_IMU)  # send request
    #time.sleep(0.009)  # wait for reply
    response = read_msp_response()  # get reply

    if response and response[0] == MSP_RAW_IMU:
        # Decode as signed 16-bit integers
        imu = struct.unpack('<9h', response[1])  # AccX, AccY, AccZ, GyroX, GyroY, GyroZ
        #imu = struct.unpack(f'<{len(response[1]) // 2}H', response[1])  # decode
        return imu
    return None

def estimate_position():
    global position_x, position_y, position_z

    while True:
        raw_imu_data = get_raw_imu()
        if raw_imu_data:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, _, _, _ = raw_imu_data

            acc = acc_y
            # Check and update X-axis
            if abs(acc) > x_accel_threshold:
                print(f"acc: {acc}")
                if acc > 0:
                    position_x += small_value_adjustment  # Increment position
                else:
                    position_x -= small_value_adjustment  # Decrement position
            
            # Update plotting data
            current_time = time.time() - start_time
            acc_x_values.append(acc)
            time_values.append(current_time)

            # Update the plot
            ax.clear()  # Clear the previous plot
            ax.set_title('Acceleration X over Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Acceleration (milli-g)')
            ax.set_ylim(-5000, 5000)  # Set y-axis limits based on expected values
            ax.plot(time_values, acc_x_values, label='acc_x', color='blue')
            ax.legend()
            plt.draw()
            plt.pause(0.01)  # Pause to allow the plot to update

        # Uncomment to control loop timing
        time.sleep(dt)

# Start the position estimation
estimate_position()

sock.close()