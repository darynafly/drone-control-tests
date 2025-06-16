import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion
from std_msgs.msg import Header
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class IMUPoseEstimator(Node):
    """
    A ROS2 node that estimates the robot's pose (position and orientation) using IMU and Odometry data.
    Subscribes to relevant topics, processes data, and publishes the pose as an Odometry message.
    """

    def __init__(self):
        """
        Initializes the IMUPoseEstimator node.
        - Sets up QoS profiles for subscriptions and publishers.
        - Creates subscriptions to the IMU and Odometry topics.
        - Initializes variables for pose estimation.
        """
        super().__init__('imu_pose_estimator')
        
        # State variables for pose estimation
        self.imu_threshold = 1.5  # Threshold to ignore small accelerations
        self.pose_step_coef = 0.015 # Step multiplication for position increase based on accelerations
        self.x = 0.0
        self.y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_imu = self.create_subscription(
            Imu, 'mavros/imu/data', self.imu_callback, qos_profile
        )
        self.subscription_odom = self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.odom_callback, qos_profile
        )

        # Publisher for the estimated pose in Odometry format
        self.pose_publisher = self.create_publisher(Odometry, 'estimated/imu_odometry', 10)

    def odom_callback(self, msg: Odometry):
        """
        Callback function for Odometry messages.
        - Extracts roll, pitch, and yaw from the Odometry message's orientation.
        """
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def euler_from_quaternion(self, q):
        """
        Converts a quaternion to Euler angles (roll, pitch, yaw).
        
        Args:
        - q (Quaternion): The quaternion to convert.

        Returns:
        - Tuple[float, float, float]: The roll, pitch, and yaw angles in radians.
        """
        t0 = 2.0 * (q.w * q.x + q.y * q.z)
        t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (q.w * q.y - q.z * q.x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion.

        Args:
        - roll (float): The roll angle in radians.
        - pitch (float): The pitch angle in radians.
        - yaw (float): The yaw angle in radians.

        Returns:
        - Quaternion: The corresponding quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU messages.
        - Transforms IMU linear accelerations from the body frame to the global frame.
        - Updates the estimated position (x, y).

        Args:
        - msg (Imu): The incoming IMU message.
        """
        # Extract linear acceleration in the body frame
        linear_accel_x = 0.0
        if abs(msg.linear_acceleration.x) > self.imu_threshold:
            linear_accel_x = -msg.linear_acceleration.x
        linear_accel_y = 0.0
        if abs(msg.linear_acceleration.y) > self.imu_threshold:
            linear_accel_y = -msg.linear_acceleration.y
        linear_accel_z = 0.0
        if abs(msg.linear_acceleration.z) > self.imu_threshold:
            linear_accel_z = msg.linear_acceleration.z # Optional, for Z updates if needed

        # Compute the rotation matrix based on roll, pitch, and yaw
        cos_roll = math.cos(self.roll)
        sin_roll = math.sin(self.roll)
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(-self.yaw)

        # Transformation matrix from body frame to global frame
        rotation_matrix = [
            [
                cos_pitch * cos_yaw,
                cos_pitch * sin_yaw,
                -sin_pitch,
            ],
            [
                sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
                sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
                sin_roll * cos_pitch,
            ],
            [
                cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw,
                cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw,
                cos_roll * cos_pitch,
            ],
        ]

        # Transform body frame accelerations to global frame
        accel_global_x = (
            rotation_matrix[0][0] * linear_accel_x +
            rotation_matrix[0][1] * linear_accel_y #+
            #rotation_matrix[0][2] * linear_accel_z
        )
        accel_global_y = (
            rotation_matrix[1][0] * linear_accel_x +
            rotation_matrix[1][1] * linear_accel_y #+
            #rotation_matrix[1][2] * linear_accel_z
        )

        # Update positions based on global frame accelerations
        self.x += self.pose_step_coef * accel_global_x
        self.y += self.pose_step_coef * accel_global_y
        # Z can be updated if needed for 3D movement
        # self.z += self.pose_step_coef * accel_global_z

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # Optional, if Z is being tracked
        odom_msg.pose.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # Publish the pose
        self.pose_publisher.publish(odom_msg)


def main(args=None):
    """
    Entry point of the program.
    - Initializes the ROS2 context and starts the IMUPoseEstimator node.
    """
    rclpy.init(args=args)

    node = IMUPoseEstimator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()