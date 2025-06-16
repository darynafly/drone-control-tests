import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class OdomRepub(Node):

    def __init__(self):
        super().__init__('odom_repub')
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_odom = self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.odom_callback, qos_profile
        )

        # Publisher for the estimated pose in Odometry format
        self.odom_publisher = self.create_publisher(Odometry, '/ground_truth_odometry', 10)

        # Store the initial position (to calculate relative positions)
        self.initial_position = None

    def odom_callback(self, msg: Odometry):
        """
        Callback function for Odometry messages.
        Adjusts the initial position to start from 0 and publishes the relative position.
        """
        if self.initial_position is None:
            # Set the initial position from the first message
            self.initial_position = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            }
            self.get_logger().info("Initial position set to: "
                                   f"x={self.initial_position['x']}, "
                                   f"y={self.initial_position['y']}, "
                                   f"z={self.initial_position['z']}")

        # Calculate relative position
        relative_x = msg.pose.pose.position.x - self.initial_position['x']
        relative_y = msg.pose.pose.position.y - self.initial_position['y']
        relative_z = 0.0  # Height set to 0

        # Update the Odometry message with the relative position
        odom_msg = msg
        odom_msg.pose.pose.position.x = relative_x
        odom_msg.pose.pose.position.y = relative_y
        odom_msg.pose.pose.position.z = relative_z

        # Publish the updated Odometry message
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    """
    Entry point of the program.
    - Initializes the ROS2 context and starts the node.
    """
    rclpy.init(args=args)

    node = OdomRepub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
