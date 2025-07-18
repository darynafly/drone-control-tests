import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

#from path_accuracy import compare_paths, path_length_difference, visualize_paths, calculate_metrics
from path_accuracy_calculations import calculate_metrics, calculate_deviations, path_length_difference, visualize_paths

import math
import numpy as np

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Desired speed
        self.desired_speed = 1.5  # m/s
        self.tolerance_2d = 1.0
        self.tolerance_z = 1.0

        square_side = 6.0
        #square_side = 3.0

        self.waypoints = [
            [0.0, 0.0, 4.0], 
            [0.0, square_side, 4.0], 
            [square_side, square_side, 4.0],
            [square_side, 0.0, 4.0],
            [0.0, 0.0, 4.0]
        ]
        
        self.update_rate = 10.

        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=qos_profile
        )

        # Publisher
        self.position_target_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # Internal state
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

        self.np_path_all = np.empty((0, 3))
        self.np_path_real = np.empty((0, 3))
        
        self.waypoints_reached = False

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

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.navigate)

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
        #self.np_path_all = np.vstack((self.np_path_all, np.array([waypoint[0], waypoint[1], waypoint[2]])))
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
            #self.np_path_real = np.vstack((self.np_path_real, 
            #                               np.array([
            #                                    round(waypoint[0],2), 
            #                                    round(waypoint[1],2), 
            #                                    round(waypoint[2],2)
            #                                ])))

            self.path_current_real.poses.append(pose)  # Add the pose to the path

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation_q = msg.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)
        self.visualise_path_current_real(self.current_position)

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

    def navigate(self):
        if self.target_waypoint_index >= len(self.waypoints):
            self.waypoints_reached = True
            # Calculate the metrics
            threshold = 0.2
            #avg_deviation, max_deviation, min_deviation, correctness_percentage = compare_paths(self.path_all, self.path_current_real, threshold)
            #path_diff = path_length_difference(self.path_all, self.path_current_real)
            accuracy, precision, recall, f1_score = calculate_metrics(self.path_all, self.path_current_real, threshold)
            mean_deviation, min_deviation, max_deviation, deviations = calculate_deviations(self.path_all, self.path_current_real)
            path_diff = path_length_difference(self.path_all, self.path_current_real)

            print(f"All waypoints reached.")
            print(f"Threshold {threshold}")

            print(f"Length difference {path_diff} m")       
            print(f"Average Deviation: {mean_deviation:.4f} m")
            print(f"Maximum Deviation: {max_deviation:.4f} m")
            print(f"Minimum Deviation: {min_deviation:.4f} m")
            print(f"Correctness: {f1_score:.2f}%")

            visualize_paths(self.path_all, self.path_current_real, threshold)
          
            return

        target_waypoint = self.waypoints[self.target_waypoint_index]
        distance_to_target_2d = math.sqrt(
            (target_waypoint[0] - self.current_position[0]) ** 2 +
            (target_waypoint[1] - self.current_position[1]) ** 2
        )

        # Check if the distance to the waypoint is within tolerance in 2D and z
        if distance_to_target_2d < self.tolerance_2d and abs(target_waypoint[2] - self.current_position[2]) < self.tolerance_z:
            self.get_logger().info(f"Reached waypoint {target_waypoint}.")
            self.stop()
            self.target_waypoint_index += 1
        else:
            self.publish_setpoint(target_waypoint)
        self.visualise_plan_path()
        self.visualise_path_all()

    def stop(self):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )

        # Set velocity to stop
        position_target.velocity.x = 0.0
        position_target.velocity.y = 0.0
        position_target.velocity.z = 0.0 

        # Publish the setpoint
        self.position_target_pub.publish(position_target)

    def publish_setpoint(self, target_waypoint):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ
        )

        # Calculate direction to waypoint
        direction_x = target_waypoint[0] - self.current_position[0]
        direction_y = target_waypoint[1] - self.current_position[1]
        direction_z = target_waypoint[2] - self.current_position[2]
        distance_3d = math.sqrt(direction_x ** 2 + direction_y ** 2 + direction_z ** 2)
        distance_3d += 0.001

        if distance_3d > 0:
            velocity_x = (direction_x / distance_3d) * self.desired_speed
            velocity_y = (direction_y / distance_3d) * self.desired_speed
            velocity_z = (direction_z / distance_3d) * self.desired_speed
        else:
            velocity_x = 0.0
            velocity_y = 0.0
            velocity_z = 0.0

        # Set velocity to move toward the waypoint
        position_target.velocity.x = velocity_x
        position_target.velocity.y = velocity_y
        position_target.velocity.z = velocity_z * 0.8

        # Calculate yaw to face the target
        yaw = math.atan2(direction_y, direction_x)
        position_target.yaw = yaw

        # Publish the setpoint
        self.position_target_pub.publish(position_target)
        #self.get_logger().info(f"Publishing velocity setpoint: vx={velocity_x:.2f}, vy={velocity_y:.2f}, vz={position_target.velocity.z:.2f}, yaw={yaw:.2f}")


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
