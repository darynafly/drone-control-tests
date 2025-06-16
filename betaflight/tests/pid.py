import socket
import struct
import math
import time
import keyboard

class DroneController:
    def __init__(self):
        # Target initial roll, pitch, yaw values
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.base_throttle = 1200  # Base throttle for hovering

        # Control gains for adjusting motor speeds
        self.roll_kp = 50
        self.pitch_kp = 50
        self.yaw_kp = 20

        # Initialize current orientation values
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # MSP command codes
        self.MSP_SET_MOTOR = 214
        self.MSP_RAW_IMU = 102
        self.MSP_YAW = 108

        # Set up TCP connection to the drone's flight controller
        host = '192.168.1.10'  # IP address of the flight controller
        port = 5761
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

    def construct_msp_message(self, command: int, data: list = []) -> bytearray:
        """Creates MSP command in expected format."""
        message = bytearray([ord('$'), ord('M'), ord('<'), len(data), command])
        checksum = len(data) ^ command
        for byte in data:
            message.append(byte)
            checksum ^= byte
        message.append(checksum)
        return message

    def send_msp_message(self, command: int, data: list = []):
        """Send MSP command to the flight controller."""
        message = self.construct_msp_message(command, data)
        self.sock.sendall(message)

    def set_motor_values(self, motor1: int, motor2: int, motor3: int, motor4: int):
        """Set throttle values for each motor."""
        motor_values = [motor1, motor2, motor3, motor4, 0, 0, 0, 0]
        print(motor1, motor2, motor3, motor4)
        packed_data = struct.pack('<8H', *motor_values)
        self.send_msp_message(self.MSP_SET_MOTOR, packed_data)

    def read_msp_response(self) -> tuple:
        """Receive and validate MSP response from the flight controller."""
        header = self.sock.recv(3)  # read header
        if header != b'$M>':
            return None
        data_length = self.sock.recv(1)[0]
        code = self.sock.recv(1)[0]
        data = self.sock.recv(data_length)
        checksum = self.sock.recv(1)[0]
        calculated_checksum = data_length ^ code
        for byte in data:
            calculated_checksum ^= byte
        if calculated_checksum != checksum:
            return None
        return (code, data)

    def get_raw_imu(self) -> tuple:
        """Retrieve and decode IMU data."""
        self.send_msp_message(self.MSP_RAW_IMU)
        response = self.read_msp_response()
        if response and response[0] == self.MSP_RAW_IMU:
            return struct.unpack('<9h', response[1])  # AccX, AccY, AccZ, GyroX, GyroY, GyroZ
        return None

    def get_yaw(self) -> float:
        """Retrieve and decode yaw data."""
        self.send_msp_message(self.MSP_YAW)
        response = self.read_msp_response()
        if response and response[0] == self.MSP_YAW:
            yaw_data = struct.unpack('<3h', response[1])
            return yaw_data[2]  # Yaw value
        return 0.0

    def calculate_orientation(self):
        """Calculate roll, pitch, yaw from IMU data."""
        imu_data = self.get_raw_imu()
        if imu_data is None:
            return

        ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
        self.current_yaw = -self.get_yaw()

        self.current_roll = math.atan2(ay, az) * 180 / math.pi
        self.current_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi

    def maintain_hover(self):
        """Control loop to maintain drone's orientation."""
        while True:
            self.calculate_orientation()

            # Calculate errors
            roll_error = self.target_roll - self.current_roll
            pitch_error = self.target_pitch - self.current_pitch
            yaw_error = self.target_yaw - self.current_yaw

            # Wrap yaw error to the range [-180, 180]
            yaw_error = (yaw_error + 180) % 360 - 180

            # Calculate motor adjustments
            roll_adjustment = int(self.roll_kp * roll_error)
            pitch_adjustment = int(self.pitch_kp * pitch_error)
            yaw_adjustment = int(self.yaw_kp * yaw_error)

            # Motor values to stabilize the drone
            motor1 = self.base_throttle + roll_adjustment + pitch_adjustment - yaw_adjustment
            motor2 = self.base_throttle + roll_adjustment - pitch_adjustment + yaw_adjustment
            motor3 = self.base_throttle - roll_adjustment + pitch_adjustment + yaw_adjustment
            motor4 = self.base_throttle - roll_adjustment - pitch_adjustment - yaw_adjustment

            # Limit motor values to the safe range
            motor1 = max(1000, min(1700, motor1))
            motor2 = max(1000, min(1700, motor2))
            motor3 = max(1000, min(1700, motor3))
            motor4 = max(1000, min(1700, motor4))

            # Stop
            if keyboard.is_pressed("s"):
                motor1 = 0
                motor2 = 0
                motor3 = 0
                motor4 = 0
                self.set_motor_values(motor1, motor2, motor3, motor4)
            else:
                # Apply motor values
                self.set_motor_values(motor1, motor2, motor3, motor4)

            if keyboard.is_pressed("q"):
                self.sock.close()
                exit()
                
            # Delay for stability
            #time.sleep(0.05)  # Adjust as needed for response time

def main():
    drone = DroneController()
    try:
        drone.maintain_hover()
    except KeyboardInterrupt:
        print("Hovering stopped.")

if __name__ == "__main__":
    main()
