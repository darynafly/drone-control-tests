import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
import numpy as np

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.height = 0.0

        self.waypoints = [
            [0.0, 0.0, self.height], 
            #[0.0, -7.0 , self.height],
            #[0.0, -7.0 , self.height],
            #[0.0, 0.0, self.height],
            [12.0, 0.0 , self.height], 
            [12.0, 0.0 , self.height],  
            #[12.0, -7.0 , self.height], 
            #[12.0, -7.0 , self.height], 
            #[0.0, -7.0 , self.height], 
            [0.0, 0.0, self.height],
        ]

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_gps_odom = self.create_subscription(
            Odometry, 'gps_shifted_odometry', self.gps_odom_callback, qos_profile
        )
        self.subscription_imu_odom = self.create_subscription(
            Odometry, 'imu_shifted_odometry', self.imu_odom_callback, qos_profile
        )

        self.publisher_planned_path = self.create_publisher(Path, 'planned_path', 10)
        self.publisher_imu_path = self.create_publisher(Path, 'imu_path', 10)
        self.publisher_gps_path = self.create_publisher(Path, 'gps_path', 10)
        
        self.current_index = 0  # Start at the first waypoint
        self.planned_path = Path()
        self.imu_path = Path()
        self.gps_path = Path()
        self.planned_path.header.frame_id = "map"
        self.imu_path.header.frame_id = "map"
        self.gps_path.header.frame_id = "map"

        # Add a IDEAL waypoints to the path
        for waypoint in self.waypoints:    
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation

            self.planned_path.header.stamp = self.get_clock().now().to_msg()
            self.planned_path.poses.append(pose)  # Add the pose to the path

        # Timer
        self.timer = self.create_timer(0.1, self.visualize)

    def gps_odom_callback(self, msg):  
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = msg.pose.pose.position.z

        self.gps_path.header.stamp = self.get_clock().now().to_msg()
        self.gps_path.poses.append(pose)  # Add the pose to the path

    def imu_odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = msg.pose.pose.position.z

        self.imu_path.header.stamp = self.get_clock().now().to_msg()
        self.imu_path.poses.append(pose)  # Add the pose to the path
          
    def visualize(self):
        # Publish pathes
        self.publisher_planned_path.publish(self.planned_path) 
        self.publisher_imu_path.publish(self.imu_path) 
        self.publisher_gps_path.publish(self.gps_path) 

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
