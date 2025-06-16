import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import math
import numpy as np


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('path_vis')

        square_side = 6.0
        self.waypoints = [
            [0.0, 0.0, 4.0], 
            [0.0, square_side, 4.0], 
            [square_side, square_side, 4.0],
            [square_side, 0.0, 4.0],
            [0.0, 0.0, 4.0]
        ]

        self.update_rate = 10.0

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.pose_subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile=qos_profile)
        
        self.current_position = [0.0, 0.0, 1.0]
        self.current_yaw = 0.0
        self.target_waypoint_index = 0

        # Publisher for the Path topic
        self.publisher_path = self.create_publisher(Path, 'planned_path_next', 10)
        self.publisher_path_all = self.create_publisher(Path, 'planned_path', 10)
        self.publisher_path_curr = self.create_publisher(Path, 'travelled_path', 10)
        self.current_index = 0  # Start at the first waypoint
        self.path = Path()
        self.path.header.frame_id = "odom"  # Set the frame ID for the path
        self.path_all = Path()
        self.path_all.header.frame_id = "odom"
        self.path_current_real = Path()
        self.path_current_real.header.frame_id = "odom"

        # Add a IDEAL waypoints to the path
        for waypoint in self.waypoints:    
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation

            self.path_all.poses.append(pose)  # Add the pose to the path
            self.path_all.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp

        self.waypoints_reached = False
        
    def visualise_plan_path(self):
        # Add a new waypoint to the path
        if self.target_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.target_waypoint_index]
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation

            self.path.poses.append(pose)  # Add the pose to the path
            self.path.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp
            self.publisher_path.publish(self.path)  # Publish the path
        else:
            self.publisher_path.publish(self.path)  # Publish the path

    def visualise_path_all(self):
        self.publisher_path_all.publish(self.path_all)  # Publish the path
            
    def visualise_path_current_real(self, waypoint):
        # Add a new waypoint to the path
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
        pose.header.frame_id = "odom"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  # Default orientation

        self.path_current_real.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp
        self.publisher_path_curr.publish(self.path_current_real)  # Publish the path

        if not self.waypoints_reached:

            self.path_current_real.poses.append(pose)  # Add the pose to the path

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation_q = msg.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)
        self.visualise_path_current_real(self.current_position)        
        self.visualise_plan_path()
        self.visualise_path_all()

    def euler_from_quaternion(self, q):
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
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
