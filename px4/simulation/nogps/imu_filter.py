import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class AltitudeToOdometryRepublisher(Node):
    def __init__(self):
        super().__init__('robot_localization_to_mavros_republisher')
        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            #'/odometry/global',
            '/robot_localization/odometry',
            self.odom_callback,
            qos_profile)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile)
        self.pose_publisher = self.create_publisher(PoseStamped, '/mavros/mocap/pose', 10)
        self.pose_vis_publisher = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.imu_publisher = self.create_publisher(Imu, '/robot_localization/imu', 10)

    def imu_callback(self, msg: Imu):
        # Create PoseStamped message
        imu_msg = msg
        if abs(msg.linear_acceleration.x) > 3.0 and abs(msg.linear_acceleration.x) < 7.0:
            imu_msg.linear_acceleration.x = -msg.linear_acceleration.x
        else:
            imu_msg.linear_acceleration.x = 0.0
        if abs(msg.linear_acceleration.y) > 3.0 and abs(msg.linear_acceleration.y) < 7.0:
            imu_msg.linear_acceleration.y = -msg.linear_acceleration.y
        else:
            imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = msg.linear_acceleration.z
        self.imu_publisher.publish(imu_msg)

    def odom_callback(self, msg: Odometry):
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        #pose_msg.header.stamp = self.get_clock().now().to_msg()
        #pose_msg.header.frame_id = "map"
        pose_msg.pose = msg.pose.pose
        #self.pose_publisher.publish(pose_msg)
        #self.pose_vis_publisher.publish(pose_msg)
        #self.get_logger().info(f"Published PoseStamped: {pose_msg}")
        
        # # Create an Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = msg.pose.pose
        #self.odom_publisher.publish(odom_msg)
        #self.get_logger().info(f"Republished Z: {odom_msg.pose.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeToOdometryRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
