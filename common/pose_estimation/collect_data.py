import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import csv
import math

class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # File to store collected data
        self.file = open('imu_odometry_data.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['imu_accel_x', 'imu_accel_y', 'imu_accel_z', 'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])

        # Subscribe to the odometry and IMU topics
        self.odom_subscription = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self.odom_callback, qos_profile
        )

        # Store IMU data
        self.imu_data = None
        self.imu_subscription = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, qos_profile
        )

    def odom_callback(self, msg: Odometry):
        if self.imu_data is not None:
            # Extract IMU and odometry data
            imu_data = self.imu_data
            accel = imu_data.linear_acceleration
            gyro = imu_data.angular_velocity

            # Extract IMU readings
            imu_accel_x = accel.x
            imu_accel_y = accel.y
            imu_accel_z = accel.z
            imu_gyro_x = gyro.x
            imu_gyro_y = gyro.y
            imu_gyro_z = gyro.z

            # Extract ground truth from odometry (position and yaw)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            roll, pitch, yaw = self.euler_from_quaternion(msg.pose.pose.orientation) # from odom msg
            #roll, pitch, yaw = self.euler_from_quaternion(imu_data.orientation) # from imu msg

            # Write data to CSV
            self.writer.writerow([imu_accel_x, imu_accel_y, imu_accel_z, imu_gyro_x, imu_gyro_y, imu_gyro_z, x, y, z, roll, pitch, yaw])

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

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
