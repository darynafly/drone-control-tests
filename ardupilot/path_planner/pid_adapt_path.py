import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from std_msgs.msg import Header
import math
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, min_output=-float('inf'), max_output=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        """Calculate PID output using the error and time delta."""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(self.min_output, min(self.max_output, output))  # Clamp to min/max bounds
        
        self.prev_error = error
        return output

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Initialize PID controllers
        self.pid_x = PID(kp=1.0, ki=0.1, kd=0.05)
        self.pid_y = PID(kp=1.0, ki=0.1, kd=0.05)
        self.pid_z = PID(kp=1.0, ki=0.1, kd=0.05)
        self.pid_yaw = PID(kp=1.0, ki=0.1, kd=0.05)

        self.desired_speed = 1.0  # m/s
        self.tolerance_2d = 1.0
        self.tolerance_z = 1.0

        square_side = 3.0
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
        
        self.position_target_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.current_position = [0.0, 0.0, 1.0]
        self.current_yaw = 0.0
        self.target_waypoint_index = 0

        self.timer = self.create_timer(1.0 / self.update_rate, self.navigate)

    def navigate(self):
        if self.target_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target_waypoint = self.waypoints[self.target_waypoint_index]
        distance_to_target_2d = math.sqrt(
            (target_waypoint[0] - self.current_position[0]) ** 2 +
            (target_waypoint[1] - self.current_position[1]) ** 2
        )

        # Calculate errors
        error_x = target_waypoint[0] - self.current_position[0]
        error_y = target_waypoint[1] - self.current_position[1]
        error_z = target_waypoint[2] - self.current_position[2]
        error_yaw = math.atan2(error_y, error_x) - self.current_yaw

        # Apply PID controllers
        dt = 1.0 / self.update_rate  # Assuming update rate is constant
        velocity_x = self.pid_x.update(error_x, dt)
        velocity_y = self.pid_y.update(error_y, dt)
        velocity_z = self.pid_z.update(error_z, dt)
        yaw_correction = self.pid_yaw.update(error_yaw, dt)

        # Check if the waypoint is reached
        if distance_to_target_2d < self.tolerance_2d and abs(error_z) < self.tolerance_z:
            self.get_logger().info(f"Reached waypoint {target_waypoint}.")
            self.target_waypoint_index += 1
            self.stop()

        else:
            self.publish_setpoint(velocity_x, velocity_y, velocity_z, yaw_correction)

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

    def publish_setpoint(self, velocity_x, velocity_y, velocity_z, yaw_correction):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ
        )

        position_target.velocity.x = velocity_x
        position_target.velocity.y = velocity_y
        position_target.velocity.z = velocity_z

        # Apply yaw correction
        position_target.yaw = self.current_yaw + yaw_correction
        self.position_target_pub.publish(position_target)

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
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


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
