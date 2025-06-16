import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import math
import joblib
import numpy as np

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        # Load the trained model
        self.get_logger().info("Loading the pose estimation model...")
        self.model = joblib.load('pose_estimation_model.pkl')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )


        # Internal state variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.last_time = None
        self.imu_data = None

        # IMU subscription
        self.subscription_imu = self.create_subscription(
            Imu,
            'mavros/imu/data',
            #'robot_localization/imu',
            self.imu_callback,
            qos_profile
        )

        # local odom subscription
        self.odom_subscription = self.create_subscription(
            Odometry, 
            "/mavros/local_position/odom", 
            self.odom_callback, 
            qos_profile
        )


        # Odometry publisher
        self.publisher = self.create_publisher(Odometry, '/estimated/imu_odometry', 10)

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


    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def odom_callback(self, odom_msg: Odometry):
        #print("odom")
        pass


    def imu_callback(self, msg: Imu):
        self.imu_data = msg
        current_time = self.get_clock().now()

        # Extract IMU data
        imu_accel_x = self.imu_data.linear_acceleration.x
        imu_accel_y = self.imu_data.linear_acceleration.y
        imu_accel_z = self.imu_data.linear_acceleration.z
        imu_gyro_x = self.imu_data.angular_velocity.x
        imu_gyro_y = self.imu_data.angular_velocity.y
        imu_gyro_z = self.imu_data.angular_velocity.z

        # Convert quaternion to yaw
        #roll, pitch, yaw = self.euler_from_quaternion(odom_msg.pose.pose.orientation) # from odom msg
        roll, pitch, yaw = self.euler_from_quaternion(self.imu_data.orientation) # from imu msg

        # Prepare input features for the model
        input_features = np.array([[imu_accel_x, imu_accel_y, imu_accel_z, imu_gyro_x, imu_gyro_y, imu_gyro_z, roll, pitch, yaw]])

        # Predict incremental pose
        delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw = self.model.predict(input_features)[0]

        # Update the pose
        self.x += delta_x
        self.y += delta_y
        self.z += delta_z
        self.roll = roll
        self.pitch = pitch
        
        self.yaw = yaw

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'map'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z

        # Set the orientation
        odom_msg.pose.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # Publish the message
        self.publisher.publish(odom_msg)
        self.get_logger().info(f"Published pose: x={self.x}, y={self.y}, yaw={self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
