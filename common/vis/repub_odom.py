import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion
from std_msgs.msg import Header
import math
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

    def odom_callback(self, msg: Odometry):
        """
        Callback function for Odometry messages.
        - Set height to 0
        """
         # Create and publish Odometry message
        odom_msg = msg

        # Pose
        odom_msg.pose.pose.position.z = 0.0

        # Publish the pose
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