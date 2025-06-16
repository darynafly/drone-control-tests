import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State, AttitudeTarget, Altitude
from mavros_msgs.srv import CommandBool, SetMode
import math
import time

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_feedback')

        # Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_callback, 10)
        
         # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.atitude_msg = AttitudeTarget()
        self.atitude_msg.thrust = 0.75  # STart thrust for hover
        self.altitude_sub = self.create_subscription(Altitude, '/mavros/altitude', self.altitude_callback, qos_profile)

        # Publishers
        self.pub_attitude = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.arming_client.wait_for_service()
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_mode_client.wait_for_service()

        # State and Timer Setup
        self.current_state = State()
        self.time_start = time.time()
        self.state = "takeoff"
        self.desired_takeoff_time = 20.0
        self.desired_move_time = 4.0
        self.desired_height = 4.0  # Target height in meters
        self.current_altitude = 0.0  
        
        self.hover = True
        self.perform_hover = False
        self.hover_change_time = None

        # Arm and Set Offboard Mode
        self.set_mode('STABILIZED')
        while not self.arm_call(True):
            self.get_logger().info("Arming the vehicle...")
            time.sleep(0.5)
        self.get_logger().info("Vehicle armed.")

        self.prearm()
        self.set_mode('OFFBOARD')

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def prearm(self):
        """Send pre-arm attitude commands."""
        for _ in range(10):
            msg = AttitudeTarget()
            msg.thrust = 0.5  # Maintain a stable hover
            self.pub_attitude.publish(msg)

    def state_callback(self, msg):
        """Update current state."""
        self.current_state = msg

    def altitude_callback(self, msg):
        """Update the current altitude from /mavros/altitude."""
        self.current_altitude = msg.relative  # relative altitude in meters
       #print(self.current_altitude)

    def set_mode(self, custom_mode):
        """Set the flight mode."""
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_call(self, value):
        """Arm or disarm the vehicle."""
        req = CommandBool.Request()
        req.value = value
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info(f"Arming response: {response}")
        else:
            self.get_logger().error("Failed to receive arming response.")
        return response

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to a quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def adjust_thrust_for_height(self):
        """Adjust thrust based on altitude to maintain the desired height."""
        if self.perform_hover:
            # Adjust thrust based on altitude feedback
            if self.hover_change_time is not None:
                #print(time.time() - self.hover_change_time)
                if abs(time.time() - self.hover_change_time) > 0.1:
                    self.atitude_msg.thrust = 0.75
                    self.hover_change_time = None
            if self.current_altitude > self.desired_height:
                    self.atitude_msg.thrust = 0.65
                    self.hover_change_time = time.time()

    def thrust_hover(self):
        """Adjust thrust based on altitude to maintain the desired height."""
        # Adjust thrust based on altitude feedback
        if self.perform_hover:
            if self.hover:
                self.atitude_msg.thrust = 0.65
            else:
                self.atitude_msg.thrust = 0.75
            self.hover = not self.hover

    def timer_callback(self):
        """State machine to execute drone movements."""
        current_time = time.time() - self.time_start

        if self.current_altitude > self.desired_height:
            self.perform_hover = True
        self.adjust_thrust_for_height()
        print(self.atitude_msg.thrust)
        if self.state == "takeoff":
            if current_time < self.desired_takeoff_time:
                self.atitude_msg.orientation = self.euler_to_quaternion(0.0, 0.0, 0.0)
                self.pub_attitude.publish(self.atitude_msg)
                #self.get_logger().info("Taking off...")
            else:
                self.time_start = time.time()
                self.state = "land"

        elif self.state == "front":
            if current_time < self.desired_move_time:
                self.atitude_msg.orientation = self.euler_to_quaternion(0.0, 0.2, 0.0)
                self.pub_attitude.publish(self.atitude_msg)
                self.get_logger().info("Moving forward...")
            else:
                self.time_start = time.time()
                self.state = "back"

        elif self.state == "back":
            if current_time < self.desired_move_time:
                self.atitude_msg.orientation = self.euler_to_quaternion(0.0, -0.2, 0.0)
                self.pub_attitude.publish(self.atitude_msg)
                self.get_logger().info("Moving backward...")
            else:
                self.time_start = time.time()
                self.state = "land"

        elif self.state == "land":
            self.atitude_msg.thrust = 0.2  # Reduced thrust to start landing
            self.atitude_msg.orientation = self.euler_to_quaternion(0.0, 0.0, 0.0)
            self.pub_attitude.publish(self.atitude_msg)
            self.get_logger().info("Landing...")
            self.set_mode('AUTO.LAND')  # Switch to land mode once descent starts

    def shutdown(self):
        """Clean up and disarm the vehicle."""
        self.set_mode('AUTO.LAND')
        self.arm_call(False)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated by user.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
