import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        # Subscribe to the odometry and IMU topics
        self.odom_subscription = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self.odom_callback, qos_profile
        )

        # Store IMU data
        self.imu_data = None
        self.imu_subscription = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, qos_profile
        )

        # Odometry publisher
        self.publisher_imu = self.create_publisher(Odometry, '/imu/angles', 10)
        self.publisher_odom = self.create_publisher(Odometry, '/odom/angles', 10)

    def odom_callback(self, odom_msg: Odometry):
        if self.imu_data is not None:
            # Extract IMU and odometry data
            imu_data = self.imu_data

            # Extract ground truth from odometry (position and yaw)
            x = odom_msg.pose.pose.position.x
            y = odom_msg.pose.pose.position.y
            z = odom_msg.pose.pose.position.z
        
            odom_roll, odom_pitch, odom_yaw = self.euler_from_quaternion(odom_msg.pose.pose.orientation) # from odom msg
            imu_roll, imu_pitch, imu_yaw = self.euler_from_quaternion(imu_data.orientation) # from imu msg

             # Publish the odometry message
            odom_msg = Odometry()
            #odom_msg.header = Header()
            #odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'map'

            # Set the odom orientation
            odom_msg.pose.pose.orientation = self.euler_to_quaternion(odom_roll, odom_pitch, odom_yaw)

            # Publish the message
            self.publisher_odom.publish(odom_msg)

            # Set the imu orientation
            odom_msg.pose.pose.orientation = self.euler_to_quaternion(imu_roll, imu_pitch, imu_yaw)

            # Publish the message
            self.publisher_imu.publish(odom_msg)

    def imu_callback(self, msg: Imu):
        self.imu_data = msg
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
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

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
