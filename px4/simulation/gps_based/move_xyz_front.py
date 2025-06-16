import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
import sys
import time
import numpy as np
from mavros_msgs.srv import CommandBool, SetMode
import math

class Setpointer(Node):

    def __init__(self, target_x, target_y, target_z):
        super().__init__('set_goal')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.goal_reach_precision = 1.0
        self.target_pos = (target_x, target_y, target_z)

        # ------- pub -------------#
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # ------- sub -------------#
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.listener_callback,
            qos_profile
        )
        self.local_pos = PoseStamped()

        # ------- clients -------------#
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        self.set_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.set_mode('POSCTL')
        self.set_mode('OFFBOARD')

        # Timer for publishing positions
        self.timer = self.create_timer(0.1, self.timer_callback)

    def set_mode(self, custom_mode):
        request = SetMode.Request()
        request.custom_mode = custom_mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.mode_sent:
            self.get_logger().info(f'Mode changed to {custom_mode} successfully.')
        else:
            self.get_logger().error(f'Failed to set mode to {custom_mode}.')

    def arm_call(self, val):
        self.req = CommandBool.Request()
        self.req.value = val
        self.get_logger().info('arm: "%s"' % val)
        self.future = self.set_arm.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        arm_resp = self.future.result()
        if arm_resp is not None:
            self.get_logger().info('Arming response: %s' % arm_resp.success)
        else:
            self.get_logger().error('Failed to receive arm response')
        return arm_resp.success

    def timer_callback(self):
        x, y, z = self.target_pos
        if not self.is_at_position(x, y, z, self.goal_reach_precision):
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z

            # Calculate the yaw angle to face the target
            yaw = math.atan2(self.target_pos[1] - self.local_pos.pose.position.y, self.target_pos[0] - self.local_pos.pose.position.x)
            quaternion = self.yaw_to_quaternion(yaw)

            msg.pose.orientation = quaternion
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Reached target position.')
            self.set_mode('AUTO.LOITER')

    def yaw_to_quaternion(self, yaw):
        """Convert a yaw angle (in radians) to a quaternion."""
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def listener_callback(self, msg):
        self.local_pos.pose.position.x = msg.pose.position.x
        self.local_pos.pose.position.y = msg.pose.position.y
        self.local_pos.pose.position.z = msg.pose.position.z

    def is_at_position(self, x, y, z, offset):
        """Check if the drone is at the given position."""
        desired = np.array((x, y, z))
        pos = np.array((self.local_pos.pose.position.x,
                        self.local_pos.pose.position.y,
                        self.local_pos.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage: python3 move_to_xyz.py <x> <y> <z>")
        return

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_z = float(sys.argv[3])

    set_goal = Setpointer(target_x, target_y, target_z)
    try:
        rclpy.spin(set_goal)
    except KeyboardInterrupt:
        pass
    finally:
        set_goal.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
