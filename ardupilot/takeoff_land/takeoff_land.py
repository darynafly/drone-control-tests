import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from time import sleep

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        
        self.takeoff_height = 0.5

        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.landing_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_guided_mode()
        self.arm()

    def arm(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')

        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_result)

    def arm_result(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone armed successfully!')
                self.set_guided_mode()
            else:
                self.get_logger().error('Failed to arm drone! Retrying...')
                sleep(1)
                self.arm()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            sleep(1)
            self.arm()

    def set_guided_mode(self):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')

        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_guided_mode_result)

    def set_guided_mode_result(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('GUIDED mode set successfully!')
                self.perform_takeoff()
            else:
                self.get_logger().error('Failed to set GUIDED mode! Retrying...')
                sleep(1)
                self.reset_sequence()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            sleep(1)
            self.reset_sequence()

    def perform_takeoff(self):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting...')

        req = CommandTOL.Request()
        req.altitude = self.takeoff_height
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self.takeoff_result)

    def takeoff_result(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Takeoff successful!')
                sleep(5)  # Simulate flight duration
                self.perform_landing()
            else:
                self.get_logger().error('Takeoff failed! Retrying...')
                sleep(1)
                self.reset_sequence()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            sleep(1)
            self.reset_sequence()

    def perform_landing(self):
        while not self.landing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Landing service not available, waiting...')

        req = CommandTOL.Request()
        req.altitude = 0.0  # Altitude is typically ignored for landing
        future = self.landing_client.call_async(req)
        future.add_done_callback(self.landing_result)

    def landing_result(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Landing successful!')
            else:
                self.get_logger().error('Landing failed! Retrying...')
                sleep(1)
                self.perform_landing()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            sleep(1)
            self.perform_landing()

    def reset_sequence(self):
        self.get_logger().info('Resetting sequence...')
        self.arm()

def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
