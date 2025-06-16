import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header
import time

class TakeoffAndMoveForward(Node):
    def __init__(self):
        super().__init__('takeoff_and_move_forward')

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # ------- Clients -------------#
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        self.set_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Timer to continuously send velocity commands
        self.timer = self.create_timer(0.1, self.timer_callback)  # Every 0.1 seconds (10 Hz)

        self.takeoff_complete = False
        self.forward_movement = False
        self.current_altitude  = 0

        # Set target altitude and forward speed
        self.target_altitude = 5.0  # Desired altitude in meters
        self.takeoff_speed = 0.5  # Vertical speed in m/s
        self.forward_speed = 0.8  # Forward speed in m/s

        self.takeoff()

    def arm_call(self, val):
        self.req = CommandBool.Request()
        self.req.value = val
        self.get_logger().info('arm: "%s"' % val)
        self.future = self.set_arm.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        arm_resp = self.future.result()
        if arm_resp is not None:
            self.get_logger().info('Arming response: %s' % arm_resp.success)
        else:
            self.get_logger().error('Failed to receive arm response')
        return arm_resp

    def set_mode(self, custom_mode):
        """Sets the flight mode of the drone."""
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff(self):
        """Send takeoff commands by publishing vertical speed in z direction."""
        self.get_logger().info("Starting takeoff...")

        self.set_mode('MANUAL')
        print('MANUAL')
        self.set_mode('STABILIZED')
        print('STABILIZED')
        self.set_mode('AUTO.TAKEOFF')
        print('TAKEOFF')
        self.set_mode('OFFBOARD')
        print('OFFBOARD')
        self.arm_call(True)
        print('arm_drone')
        #time.sleep(1)

        msg = Twist()

        # Start ascending (z axis)
        while self.current_altitude < self.target_altitude:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = self.takeoff_speed  # Ascend at 0.5 m/s
            self.velocity_publisher.publish(msg)

            # Simulate altitude change (replace with real altitude estimation in production)
            self.current_altitude += self.takeoff_speed * 0.1
            #self.get_logger().info(f"Altitude: {self.current_altitude:.2f} m")

            time.sleep(0.1)  # Publish at 10 Hz

        self.get_logger().info(f"Reached target altitude: {self.target_altitude} meters.")
        self.takeoff_complete = True

    def move_forward(self):
        """Send forward velocity commands after takeoff."""
        self.get_logger().info(f"Moving forward at {self.forward_speed} m/s.")
        msg = Twist()

        while self.forward_movement:
            msg.linear.x = self.forward_speed  # Move forward at 1.0 m/s
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            # No rotation (angular velocity is zero)
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            self.velocity_publisher.publish(msg)
            time.sleep(0.1)  # Publish at 10 Hz

    def timer_callback(self):
        """Callback function to control the flow of the process."""
        if self.current_altitude >= self.target_altitude:
            self.move_forward()
            self.forward_movement = True

def main(args=None):
    rclpy.init(args=args)
    takeoff_node = TakeoffAndMoveForward()

    try:
        # Start the takeoff and forward movement process
        rclpy.spin(takeoff_node)
    except KeyboardInterrupt:
        pass
    finally:
        takeoff_node.arm_call(False)  # Disarm the drone on shutdown
        takeoff_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
