import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PoseStamped
from mavros_msgs.msg import AttitudeTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import math
import land

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
        self.pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.timer = self.create_timer(0.1, self.publish_attitude)
        
        self.current_position = [0, 0, 5.0]
        
        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=qos_profile)

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
        msg = AttitudeTarget()
        msg.thrust = 1.0  # how fast to move

        # Roll, pitch, and yaw
        roll = 0.0   
        pitch = -0.6
        yaw = 0.0 
        
        if self.current_position[2] < 4.0:
            pitch = 0.2
        
        # Convert to quaternion
        msg.orientation = euler_to_quaternion(roll, pitch, yaw)
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
