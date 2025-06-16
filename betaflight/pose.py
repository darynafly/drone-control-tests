import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

import socket
import struct
import time
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        ### Connection
        # Connect to a Betaflight drone via TCP
        host = '192.168.1.2'  # IP address of the drone
        port = 5761  # port typically used for TCP-based connections with Betaflight

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

        ### MSP commands
        # out messages usually start with 1 (1**)
        self.MSP_STATUS = 101
        self.MSP_RAW_IMU = 102
        # in messages usually start with 2 (2**)
        self.MSP_SET_MOTOR = 214

        self.MSP_YAW = 108

        # Constants
        self.yaw_dt = 0.001
        self.small_accel_threshold = 30.0  # Threshold to consider acceleration as small

        # orientation estimation
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0

        # Kalman Parameters

        # Time step
        self.dt = 0.01
        process_noise = 15.5
        measurement_noise = 7.

        # State vector [position_x, position_y, position_z, velocity_x, velocity_y, velocity_z]
        self.x = np.zeros((6, 1))

        # State transition matrix
        self.A = np.eye(6)
        for i in range(3):
            self.A[i, i + 3] = self.dt  # Position update from velocity

        # Control matrix for accelerometer input (assumes constant acceleration within dt)
        self.B = np.zeros((6, 3))
        for i in range(3):
            self.B[i + 3, i] = self.dt

        # Process noise covariance (estimated noise in the system)
        self.Q = process_noise * np.eye(6)

        # Measurement noise covariance (estimated sensor noise)
        self.R = measurement_noise * np.eye(3)

        # Measurement matrix to relate state to position measurements
        self.H = np.zeros((3, 6))
        for i in range(3):
            self.H[i, i] = 1

        # Initial covariance matrix
        self.P = np.eye(6)

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    ### Main functions
    def construct_msp_message(self, command: int, data: list = []) -> bytearray:
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

    def send_msp_message(self, command: int, data: list = []):
        """Send prepared command via serial

        Args:
            command (int): MSP command
            data (list, optional): list of params. Defaults to [].
        """
        message = self.construct_msp_message(command, data)
        self.sock.sendall(message)

    ### Set motors values functions
    def set_motor_values(self, motor1: int, motor2: int, motor3: int, motor4: int):
        """Set throttle per motor

        Args:
            motor1 (int): motor 1 throttle
            motor2 (int): motor 2 throttle
            motor3 (int): motor 3 throttle
            motor4 (int): motor 4 throttle
        """
        motor_values = [motor1, motor2, motor3, motor4, 0, 0, 0, 0]
        self.send_msp_message(self.MSP_SET_MOTOR, struct.pack('<8H', *motor_values))
        
    def read_msp_response(self) -> tuple:
        """Read data

        Returns:
            tuple: command code and its values
        """
        header = self.sock.recv(3)  # read header
        if header != b'$M>':
            return None
        # read body
        data_length = self.sock.recv(1)[0]
        code = self.sock.recv(1)[0]
        data = self.sock.recv(data_length)
        # read tail
        checksum = self.sock.recv(1)[0]
        # check if the command is of the expected length
        calculated_checksum = data_length ^ code
        for byte in data:
            calculated_checksum ^= byte
        if calculated_checksum != checksum:
            return None

        return (code, data)

    def get_raw_imu(self) -> tuple:
        """Read raw IMU data and decode it as signed 16-bit integers.

        Returns:
            tuple | None: tuple - processed data, None - received data doesn't correspond to sent command
        """
        self.send_msp_message(self.MSP_RAW_IMU)  # send request
        #time.sleep(0.009)  # wait for reply
        response = self.read_msp_response()  # get reply

        if response and response[0] == self.MSP_RAW_IMU:
            # Decode as signed 16-bit integers
            imu = struct.unpack('<9h', response[1])  # AccX, AccY, AccZ, GyroX, GyroY, GyroZ
            #imu = struct.unpack(f'<{len(response[1]) // 2}H', response[1])  # decode
            return imu
        return None
 
    def get_yaw(self) -> tuple:
        """Read yaw data and decode it.

        Returns:
            tuple | None: tuple - processed data, None - received data doesn't correspond to sent command
        """
        self.send_msp_message(self.MSP_YAW)  # send request
        #time.sleep(0.009)  # wait for reply
        response = self.read_msp_response()  # get reply

        if response and response[0] == self.MSP_YAW:
            # Decode as signed 16-bit integers
            yaw = struct.unpack(f'<{len(response[1]) // 2}H', response[1])  # decode
            #imu = struct.unpack(f'<{len(response[1]) // 2}H', response[1])  # decode
            return yaw[2]
        return None
    
    def get_accel(self) -> tuple:
        """Read raw IMU data and decode it as signed 16-bit integers.

        Returns:
            tuple | None: tuple - processed data, None - received data doesn't correspond to sent command
        """
        self.send_msp_message(self.MSP_RAW_IMU)  # send request
        response = self.read_msp_response()  # get reply

        if response and response[0] == self.MSP_RAW_IMU:
            # Decode as signed 16-bit integers
            imu = struct.unpack('<9h', response[1])  # AccX, AccY, AccZ, GyroX, GyroY, GyroZ
            #imu = struct.unpack(f'<{len(response[1]) // 2}H', response[1])  # decode
            return imu[:3] # AccX, AccY, AccZ
        return None

    def calc_orientations(self):
        # Get the next Z-axis acceleration value
        imu_data = self.get_raw_imu()

        ax = imu_data[1]
        ay = imu_data[2]
        az = imu_data[5]
        self.yaw = -self.get_yaw()

        self.roll = math.atan2(ay, az) * 180 / math.pi
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
        
        # Calc yaw from accelerations on Z
        #if abs(az) < self.small_accel_threshold:
        #    # Keep the current angle if acceleration is small
        #    yaw_change = 0
        #    #print(f"Acceleration is small (|{az}| < {small_accel_threshold}). Keeping current yaw: {yaw:.2f} degrees.")
        #else:
        #    # Estimate change in yaw based on acceleration
        #    yaw_change = az * self.yaw_dt 
        #    self.yaw += yaw_change

    def euler_to_quaternion(self, yaw, pitch, roll):
        """Convert Euler angles (in degrees) to a quaternion."""
        # Convert degrees to radians
        yaw_rad = np.deg2rad(yaw)
        pitch_rad = np.deg2rad(pitch)
        roll_rad = np.deg2rad(roll)

        # Calculate the quaternion components
        cy = np.cos(yaw_rad * 0.5)
        sy = np.sin(yaw_rad * 0.5)
        cp = np.cos(pitch_rad * 0.5)
        sp = np.sin(pitch_rad * 0.5)
        cr = np.cos(roll_rad * 0.5)
        sr = np.sin(roll_rad * 0.5)

        q = [
            cy * cp * cr + sy * sp * sr,  # qx
            sy * cp * cr - cy * sp * sr,  # qy
            cy * sp * cr + sy * cp * sr,  # qz
            cy * cp * sr - sy * sp * cr   # qw
        ]
        return q

    def predict(self, u):
        # Predict the state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u.reshape(3, 1))  # Reshape u to (3, 1)
        
        # Predict the covariance
        self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

    def update(self, z):
        # Measurement residual
        y = z - np.dot(self.H, self.x)
        
        # Residual covariance
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        
        # Kalman gain
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        
        # Update the state
        self.x = self.x + np.dot(K, y)
        
        # Update the covariance
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

    def get_position(self):
        # Return only the current position estimate (x, y, z)
        return self.x[0:3].flatten()

    def timer_callback(self):
        self.calc_orientations()
        
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'  # Set your frame ID as necessary

        quaternion = self.euler_to_quaternion(self.yaw, self.pitch, self.roll)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        # Kalman pose estimate
        acceleration = np.array(self.get_accel())  # Example: get acceleration data [ax, ay, az]

        self.predict(u=acceleration)  # Predict with acceleration as control input
        measured_position = np.zeros((3, 1))  # Assuming no external measurement for this example
        self.update(z=measured_position)  # Update with "no measurement" to simulate pure integration
        
        # Get and print only the current x, y, z position estimate
        position = self.get_position()
        #print(f"Position (x, y, z): {position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}")

        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]

        # Publish the IMU message
        self.publisher_.publish(msg)

        self.get_logger().info('Publishing data')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    pose_publisher.sock.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
