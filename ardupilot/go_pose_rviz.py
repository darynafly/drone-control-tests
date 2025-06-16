import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class GoalFollower(Node):
    def __init__(self):
        super().__init__('goal_follower')

        # Desired speed and tolerances
        self.desired_speed = 1.0  # m/s
        self.tolerance_2d = 1.0
        self.tolerance_z = 1.0

        # Internal state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.goal_position = None
        self.goal_yaw = 0.0  # To store the yaw from the goal pose

        # QoS settings for subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )

        # Subscribers
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_profile
        )

        # Publisher
        self.position_target_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # Timer for navigation
        self.timer = self.create_timer(0.1, self.navigate)

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation_q = msg.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)

    def goal_callback(self, msg):
        self.goal_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        goal_orientation_q = msg.pose.orientation
        _, _, self.goal_yaw = self.euler_from_quaternion(goal_orientation_q)
        self.get_logger().info(f"New goal received: {self.goal_position}, Yaw: {self.goal_yaw}")

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
        if not self.goal_position:
            self.get_logger().info("No goal received yet.")
            return

        distance_to_goal_2d = math.sqrt(
            (self.goal_position[0] - self.current_position[0]) ** 2 +
            (self.goal_position[1] - self.current_position[1]) ** 2
        )

        distance_to_goal_z = abs(self.goal_position[2] - self.current_position[2])

        # Check if within tolerance
        if distance_to_goal_2d < self.tolerance_2d: # and distance_to_goal_z < self.tolerance_z:
            self.get_logger().info("Goal reached!")
            self.stop()
        else:
            self.publish_setpoint(self.goal_position, self.goal_yaw)

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
        position_target.velocity.x = 0.0
        position_target.velocity.y = 0.0
        position_target.velocity.z = 0.0
        self.position_target_pub.publish(position_target)

    def publish_setpoint(self, goal_position, goal_yaw):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ
        )

        # Calculate velocity
        direction_x = goal_position[0] - self.current_position[0]
        direction_y = goal_position[1] - self.current_position[1]
        direction_z = goal_position[2] - self.current_position[2]

        distance = math.sqrt(direction_x ** 2 + direction_y ** 2) + 0.001
        velocity_x = (direction_x / distance) * self.desired_speed
        velocity_y = (direction_y / distance) * self.desired_speed

        position_target.velocity.x = velocity_x
        position_target.velocity.y = velocity_y

        # Set the yaw from the goal orientation
        position_target.yaw = goal_yaw

        self.position_target_pub.publish(position_target)


def main(args=None):
    rclpy.init(args=args)
    goal_follower = GoalFollower()
    rclpy.spin(goal_follower)
    goal_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()