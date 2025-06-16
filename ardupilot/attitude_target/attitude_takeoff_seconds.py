import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Quaternion, PoseStamped, Twist
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

import math
from time import sleep, time

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from takeoff_land import land

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion.
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        self.pub_rpy = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.pub_vel = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        self.max_retries = 5
        self.armed = False
        self.arm()

        self.start_time = time()
        self.desired_takeoff_time = 2.0

        self.timer = self.create_timer(0.01, self.publish_attitude)
        
        self.current_position = [0, 0, 5.0]

        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=qos_profile)

    def arm(self):
        retries = 0
        while retries < self.max_retries:
            if not self.arming_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Arming service not available, waiting...')
                sleep(1)
                retries += 1
                continue

            self.get_logger().info('Sending arming request...')
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)

            try:
                self.get_logger().info('Drone armed successfully!')
                self.set_guided_mode()
                self.armed = True
            except Exception as e:
                self.get_logger().error(f'Arming service call failed: {e}')
            
            retries += 1
            sleep(1)
        
        self.get_logger().error('Max retries reached. Failed to arm drone.')

    def set_guided_mode(self):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')
            sleep(1)

        self.get_logger().info('Setting mode to GUIDED_NOGPS...')
        req = SetMode.Request()
        req.custom_mode = 'GUIDED_NOGPS'
        self.set_mode_client.call_async(req)

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation_q = msg.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.current_yaw = yaw

    def euler_from_quaternion(self, q):
        """Convert a quaternion to euler."""
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def publish_attitude(self):
        
        if self.armed:
            if time() - self.start_time < self.desired_takeoff_time:
                msg = AttitudeTarget()
                msg.thrust = 0.8 # Maintain a stable hover

                # Roll, pitch, and yaw
                roll = 0.0  # Positive value moves right; negative moves left
                pitch = 0.0  # Keep the pitch neutral to avoid forward/backward movement
                yaw = 0.0  # Maintain current yaw

                # Convert to quaternion
                msg.orientation = euler_to_quaternion(roll, pitch, yaw)
                self.pub_rpy.publish(msg)

                print("takeoff")
                # Optional: Command velocity to maintain stability and fine-tune movement
                #cmd_vel = Twist()
                #cmd_vel.linear.x = 0.0  # No forward/backward motion
                #cmd_vel.linear.y = 0.5  # Positive value moves right; negative moves left
                #cmd_vel.linear.z = 0.0  # Maintain altitude
                #self.pub_vel.publish(cmd_vel)
            else:
                print("land")
                land_node = land.DroneLand()
                land_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
