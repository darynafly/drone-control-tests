import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Desired speed
        self.desired_speed = 5.0  # m/s
        self.tolerance = self.desired_speed * 0.8

        self.height = 1.5
        self.waypoints =  [
            [0.0, 0.0], 
            [0.0, 10.0], 
            [10.0, 10.0], 
            [10.0, 0.0], 
            [0.0, 0.0]
        ]
    
        self.update_rate = 30.

        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
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
        self.current_position = [0.0, 0.0]
        self.current_yaw = 0.0
        self.target_waypoint_index = 0

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.navigate)

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y]
        orientation_q = msg.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)

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

    def yaw_to_quaternion(self, yaw):
        """Convert a yaw angle (in radians) to a quaternion."""
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def navigate(self):
        if self.target_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target_waypoint = self.waypoints[self.target_waypoint_index]
        distance_to_target = math.sqrt(
            (target_waypoint[0] - self.current_position[0]) ** 2 +
            (target_waypoint[1] - self.current_position[1]) ** 2
        )

        if distance_to_target < self.tolerance:
            self.get_logger().info(f"Reached waypoint {target_waypoint}.")
            self.stop()
            self.target_waypoint_index += 1
        else:
            self.publish_setpoint(target_waypoint)

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
        distance = math.sqrt(direction_x ** 2 + direction_y ** 2)

        if distance > 0:
            velocity_x = (direction_x / distance) * self.desired_speed
            velocity_y = (direction_y / distance) * self.desired_speed
        else:
            velocity_x = 0.0
            velocity_y = 0.0

        # Set velocity to move toward the waypoint
        position_target.velocity.x = velocity_x
        position_target.velocity.y = velocity_y
        position_target.velocity.z = 0.0  # Maintain current altitude

        # Calculate yaw to face the target
        yaw = math.atan2(direction_y, direction_x)
        position_target.yaw = yaw

        # Publish the setpoint
        self.position_target_pub.publish(position_target)
        self.get_logger().info(f"Publishing velocity setpoint: vx={velocity_x:.2f}, vy={velocity_y:.2f}, yaw={yaw:.2f}")


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
