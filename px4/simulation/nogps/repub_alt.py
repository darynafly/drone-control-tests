import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class AltitudeToOdometryRepublisher(Node):
    def __init__(self):
        super().__init__('altitude_to_odometry_republisher')
        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.altitude_callback,
            qos_profile)
        self.publisher = self.create_publisher(Odometry, '/relative_altitude', 10)
        
    def altitude_callback(self, msg: Altitude):
        # Create an Odometry message
        odom_msg = Odometry()
        
        # Fill the header from the Altitude message
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'

        # Fill only the Z value from the relative field
        odom_msg.pose.pose.position.z = msg.relative
        
        # Publish the Odometry message
        self.publisher.publish(odom_msg)
        #self.get_logger().info(f"Republished Z: {odom_msg.pose.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeToOdometryRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
