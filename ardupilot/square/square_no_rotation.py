import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Parameters
        #self.declare_parameter('waypoints', [])
        self.declare_parameter('tolerance', 0.1)
        self.declare_parameter('update_rate', 10.0)

        self.height = 1.5
        self.waypoints =  [
            [0.0, 0.0], 
            [0.0, 2.0], 
            [2.0, 2.0], 
            [2.0, 0.0], 
            [0.0, 0.0]
        ]
        
        self.tolerance = self.get_parameter('tolerance').value
        self.update_rate = self.get_parameter('update_rate').value

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
            self.get_logger().info("All waypoints reached.")
            return

        target_waypoint = self.waypoints[self.target_waypoint_index]
        distance_to_target = math.sqrt(
            (target_waypoint[0] - self.current_position[0]) ** 2 +
            (target_waypoint[1] - self.current_position[1]) ** 2
        )

        if distance_to_target < self.tolerance:
            self.get_logger().info(f"Reached waypoint {target_waypoint}.")
            self.target_waypoint_index += 1
        else:
            self.publish_setpoint(target_waypoint)

    def publish_setpoint(self, target_waypoint):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        # Set position
        position_target.position.x = target_waypoint[0]
        position_target.position.y = target_waypoint[1]
        position_target.position.z = self.height

        # Set yaw (optional: keep facing forward)
        position_target.yaw = 0.0

        self.position_target_pub.publish(position_target)
        self.get_logger().info(f"Publishing setpoint to {target_waypoint}")

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
