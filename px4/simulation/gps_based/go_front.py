import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header
import math
import time
import sys

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class MoveTo(Node):
    def __init__(self, target_x, target_y, speed, threashold=1.0):
        super().__init__('move_forward')

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        self.set_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscription to get current position
        self.subscription = self.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.listener_callback,
            qos_profile
        )
        self.local_pos = Odometry()

        # Timer to send velocity commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.takeoff_complete = True
        self.forward_movement = False
        self.current_altitude = 0
        self.forward_speed = speed  # Forward speed in m/s
        self.threashold = threashold

        # Target position (example values)
        self.target_x = target_x  # Target X coordinate
        self.target_y = target_y   # Target Y coordinate
        self.reached_target = False

        self.set_mode('OFFBOARD')
        print('OFFBOARD')

    def listener_callback(self, msg):
        """Callback to update the current position of the drone."""
        print(f"x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y} ")
        self.local_pos = msg

    def calculate_distance_to_target(self):
        """Calculate Euclidean distance to the target position."""
        dx = self.target_x - self.local_pos.pose.pose.position.x
        dy = self.target_y - self.local_pos.pose.pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def arm_call(self, val):
        req = CommandBool.Request()
        req.value = val
        self.get_logger().info('arm: "%s"' % val)
        future = self.set_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        arm_resp = future.result()
        if arm_resp is not None:
            self.get_logger().info('Arming response: %s' % arm_resp.success)
        else:
            self.get_logger().error('Failed to receive arm response')
        return arm_resp.success

    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move_forward(self):
        """Send forward velocity commands if not at target."""
        if not self.reached_target:
            msg = Twist()
            msg.linear.x = self.forward_speed
            self.velocity_publisher.publish(msg)

    def timer_callback(self):
        """Check if target reached and control forward movement."""
        if self.takeoff_complete and not self.reached_target:
            distance = self.calculate_distance_to_target()
            if distance <= self.threashold:  # Threshold distance to stop
                self.reached_target = True
                self.get_logger().info("Target reached. Stopping movement.")
                stop_msg = Twist()  # Stop moving
                self.velocity_publisher.publish(stop_msg)
            else:
                self.move_forward()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) !=5:
        print("Usage: python3 move_to_xyz.py <x> <y> <speed> <threashold>")
        return

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    speed = float(sys.argv[3])
    threashold = float(sys.argv[4])

    takeoff_node = MoveTo(target_x, target_y, speed, threashold)

    try:
        rclpy.spin(takeoff_node)
    except KeyboardInterrupt:
        pass
    finally:
        takeoff_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()