import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
import tf_transformations  # For quaternion operations
import numpy as np  # For rotation matrices

class OdomRepub(Node):

    def __init__(self, yaw_shift):
        super().__init__('odom_repub')

        self.yaw_shift = yaw_shift  # Yaw shift in radians

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_odom = self.create_subscription(
            Odometry, 'estimated/imu_odometry', self.odom_callback, qos_profile
        )

        self.odom_publisher = self.create_publisher(Odometry, '/imu_shifted_odometry', 10)

        self.initial_position = None
        self.initial_orientation = None
        self.rotation_matrix = None
        self.yaw_shift_matrix = self.create_yaw_shift_matrix(yaw_shift)

    def odom_callback(self, msg: Odometry):
        """
        Callback for Odometry messages. Aligns movement and orientation to the initial reference frame
        and applies a yaw shift.
        """
        if self.initial_position is None:
            # Set the initial position and orientation
            self.initial_position = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            }
            self.initial_orientation = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )

            # Create a rotation matrix from the initial orientation
            self.rotation_matrix = self.quaternion_to_rotation_matrix(self.initial_orientation)
            self.get_logger().info("Initial position and orientation set.")

        # Calculate relative position
        relative_x = msg.pose.pose.position.x - self.initial_position['x']
        relative_y = msg.pose.pose.position.y - self.initial_position['y']
        relative_z = 0.0  # Ignore height differences

        # Transform position to the initial frame
        relative_position = np.array([relative_x, relative_y, relative_z])
        transformed_position = self.rotation_matrix.T @ relative_position

        # Apply the yaw shift
        final_position = self.yaw_shift_matrix @ transformed_position

        # Update the Odometry message with the transformed position
        odom_msg = msg
        odom_msg.pose.pose.position.x = final_position[0]
        odom_msg.pose.pose.position.y = final_position[1]
        odom_msg.pose.pose.position.z = final_position[2]

        # Adjust the orientation relative to the initial orientation
        current_orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        relative_orientation = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_conjugate(self.initial_orientation),
            current_orientation
        )

        # Apply yaw shift to the orientation
        yaw_shift_quaternion = tf_transformations.quaternion_from_euler(0, 0, self.yaw_shift)
        adjusted_orientation = tf_transformations.quaternion_multiply(
            yaw_shift_quaternion,
            relative_orientation
        )

        odom_msg.pose.pose.orientation.x = adjusted_orientation[0]
        odom_msg.pose.pose.orientation.y = adjusted_orientation[1]
        odom_msg.pose.pose.orientation.z = adjusted_orientation[2]
        odom_msg.pose.pose.orientation.w = adjusted_orientation[3]

        # Publish the updated Odometry message
        self.odom_publisher.publish(odom_msg)

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion):
        """
        Converts a quaternion to a 3x3 rotation matrix.
        """
        x, y, z, w = quaternion
        return np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
        ])

    @staticmethod
    def create_yaw_shift_matrix(yaw_shift):
        """
        Creates a 3x3 rotation matrix for a given yaw shift in radians.
        """
        cos_yaw = np.cos(yaw_shift)
        sin_yaw = np.sin(yaw_shift)
        return np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])

def main(args=None):
    """
    Entry point of the program.
    """
    rclpy.init(args=args)

    # Define the yaw shift in radians
    #yaw_shift = np.radians(70)  # squares_unticlockwise/
    #yaw_shift = np.radians(80)  # squares_clockwise/
    #yaw_shift = np.radians(76)  # squares_clockwise_2/
    #yaw_shift = np.radians(70)  # squares_unticlockwise_2/
    #yaw_shift = np.radians(0)  # lines/
    yaw_shift = np.radians(70)  # lines_4/

    node = OdomRepub(yaw_shift)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
