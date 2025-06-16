import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header, Float64, String

import tf2_ros

import math
import numpy as np
import time

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
        self.imu_x_threshold_min = 0.1  # Threshold to ignore small accelerations
        self.imu_y_threshold_min = 0.1
        self.imu_threshold_max = 5.0  # Threshold to cut big accelerations
        self.pose_step_coef = 0.05 # Step multiplication for position increase based on accelerations
        self.x = 0.0
        self.y = 0.0
        
        self.use_twist_pose = False
        self.imu_twist_threshold_min = 0.2  # Threshold to ignore small twists
        self.pose_twist_step_coef = 0.1 # Step multiplication for position increase based on twist

        self.z_step_coef = 0.001
        self.imu_z_threshold_min = 0.05
        
        self.pose_twist_x = 0.0
        self.pose_twist_y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.twist_roll = 0.0
        self.twist_pitch = 0.0
        self.twist_yaw = 0.0
        self.relative_altitude=0.0
        self.relative_altitude_array=[]
        self.drone_state = ""

        self.prev_twist_x = None
        self.prev_twist_y = 0.0
        self.prev_twist_timer = None

        self.velocity_z = 0.0  # Estimated vertical velocity
        self.altitude_from_accel = 0.0  # Estimated altitude from acceleration
        self.prev_time = None  # For time difference computation
        self.alt_offset = 0.0
        self.alt_offset_set = False
        self.start_z_accel = True

        self.x_offset = []
        self.y_offset = []
        self.twist_x_offset = []
        self.twist_y_offset = []
        self.start_x_accel = None
        self.start_y_accel = None

        # Publisher for the estimated pose in Odometry format
        self.imu_estimated_odom_publisher = self.create_publisher(Odometry, '/estimated/imu_simple_odometry', 10) 
        self.out_odom_publisher = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.pose_mocap_publisher = self.create_publisher(PoseStamped, '/mavros/mocap/pose', 10)
        self.pose_vision_publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_imu = self.create_subscription(
            Imu, 
            'mavros/imu/data', 
            self.imu_callback, 
            qos_profile
        )
        self.subscription_odom = self.create_subscription(
            Odometry, 
            '/mavros/global_position/local', 
            self.odom_callback, 
            qos_profile
        )
        self.subscription_alt = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.altitude_callback,
            qos_profile)
        self.subscription_control = self.create_subscription(
            String, 
            '/drone/state', 
            self.control_callback, 
            qos_profile
        )

        rate = 30
        self.timer = self.create_timer(1.0 / rate, self.estimate_pose)

    def control_callback(self, msg: String):
        self.drone_state = msg.data

    def altitude_callback(self, msg):
        """ Processes altitude sensor readings """
        if not self.alt_offset_set:
            self.alt_offset = -msg.data
            self.alt_offset_set = True

        rel_alt = self.alt_offset + msg.data
        rel_alt = max(0.0, rel_alt)  # Prevent negative altitude
        self.relative_altitude = rel_alt

        # Smooth altitude readings
        #if self.altitude_from_accel > 0:
        #    if rel_alt > self.relative_altitude:
        #        self.relative_altitude_array.append(rel_alt)
        #if self.altitude_from_accel < 0:
        #    if rel_alt < self.relative_altitude:
        #        self.relative_altitude_array.append(rel_alt)
        #if len(self.relative_altitude_array) > 10:
        #    self.relative_altitude = np.mean(self.relative_altitude_array)# / 1.5
        #    self.relative_altitude_array = self.relative_altitude_array[5:]

    def sameSign(self, x, y):
        if (x*y > 0):
            return True
        else:
            return False

    def odom_callback(self, msg: Odometry):
        """
        Callback function for Odometry messages.
        - Extracts roll, pitch, and yaw from the Odometry message's orientation.
        """
        #self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        if self.use_twist_pose:
            twist_y = 0.0
            twist_x = msg.twist.twist.linear.x

            if self.prev_twist_x is not None:
                if not self.sameSign(self.prev_twist_x, twist_x):
                    if self.prev_twist_timer is None:
                        self.prev_twist_timer = time.time()
                    if time.time() - self.prev_twist_timer > 2.0:
                        self.prev_twist_x = None
                        self.prev_twist_timer = None
                    twist_x = 0.0

            # Extract linear acceleration in the body frame
            if abs(twist_x) > self.imu_twist_threshold_min:
                if self.prev_twist_x is None:
                    self.prev_twist_x = twist_x
                    #self.imu_twist_threshold_min = twist_x / 2.0

                # Compute the rotation matrix based on roll, pitch, and yaw
                self.twist_roll, self.twist_pitch, self.twist_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
                cos_roll = math.cos(self.twist_roll)
                sin_roll = math.sin(self.twist_roll)
                cos_pitch = math.cos(self.twist_pitch)
                sin_pitch = math.sin(self.twist_pitch)
                cos_yaw = math.cos(self.twist_yaw)
                sin_yaw = math.sin(-self.twist_yaw)

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
                twist_global_x = (
                    rotation_matrix[0][0] * twist_x +
                    rotation_matrix[0][1] * twist_y #+
                    #rotation_matrix[0][2] * linear_accel_z
                )
                twist_global_y = (
                    rotation_matrix[1][0] * twist_x +
                    rotation_matrix[1][1] * twist_y #+
                    #rotation_matrix[1][2] * linear_accel_z
                )
                #accel_global_z =

                # Update positions based on global frame accelerations
                self.pose_twist_x += self.pose_twist_step_coef * twist_global_x
                self.pose_twist_y += self.pose_twist_step_coef * twist_global_y


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
        g = 9.8  # Gravity
        accel_z = msg.linear_acceleration.z - g  # Remove gravity influence
        self.altitude_from_accel = accel_z #*  self.z_step_coef  # Integrate velocity to get altitude
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.orientation)

        if self.start_x_accel is None:
            self.x_offset.append(-msg.linear_acceleration.x)
            if len(self.x_offset) > 5:
                self.start_x_accel = np.mean(self.x_offset)
        if self.start_y_accel is None:
            self.y_offset.append(-msg.linear_acceleration.y)
            if len(self.y_offset) > 5:
                self.start_y_accel = np.mean(self.y_offset)

        if self.start_x_accel and self.start_y_accel and self.start_z_accel:
            current_accel_x = -msg.linear_acceleration.x - self.start_x_accel
            current_accel_y = -msg.linear_acceleration.y - self.start_y_accel

            self.x_offset.append(current_accel_x)
            self.y_offset.append(current_accel_y)

            if len(self.x_offset) > 5:
                self.x_offset = self.x_offset[1:]
                self.y_offset = self.y_offset[1:]

            linear_accel_x = np.mean(self.x_offset)
            linear_accel_y = np.mean(self.y_offset)

            # Extract linear acceleration in the body frame
            if abs(linear_accel_x) < self.imu_x_threshold_min:
                linear_accel_x = 0.0
            #if abs(linear_accel_x) > self.imu_threshold_max:
            #    if linear_accel_x < 0:
            #        linear_accel_x = -self.imu_threshold_max
            #    else:
            #        linear_accel_x = self.imu_threshold_max

            if abs(linear_accel_y) < self.imu_y_threshold_min:
                linear_accel_y = 0.0
            #if abs(linear_accel_y) > self.imu_threshold_max:
            #    if linear_accel_y < 0:
            #        linear_accel_y = -self.imu_threshold_max
            #    else:
            #        linear_accel_y = self.imu_threshold_max
            
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
            #accel_global_z =

            # Update positions based on global frame accelerations
            self.x += self.pose_step_coef * accel_global_x
            self.y += self.pose_step_coef * accel_global_y

    def estimate_pose(self):
        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.relative_altitude
        odom_msg.pose.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        #print(f"Current x: {self.x}, y:{self.y}, z:{self.relative_altitude}, yaw: {self.yaw}")

        # Publish the pose
        self.imu_estimated_odom_publisher.publish(odom_msg)

        time_now = self.get_clock().now().to_msg()
        # Create and publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = time_now
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.relative_altitude # or change with IMu z estimation
        pose_msg.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        self.pose_vision_publisher.publish(pose_msg)

        print(f"IMU estimated x: {self.x}, y:{self.y}, z:{self.relative_altitude}, yaw: {self.yaw}")

        if self.use_twist_pose:
            # Create and publish Odometry message
            odom_msg = Odometry()
            odom_msg.header = Header()
            odom_msg.header.stamp = time_now
            odom_msg.header.frame_id = 'map'

            odom_msg.pose.pose.position.x = self.pose_twist_x
            odom_msg.pose.pose.position.y = self.pose_twist_y
            odom_msg.pose.pose.position.z = self.relative_altitude # or change with IMu z estimation
            odom_msg.pose.pose.orientation = self.euler_to_quaternion(self.twist_roll, self.twist_pitch, self.twist_yaw)
            self.out_odom_publisher.publish(odom_msg)


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
