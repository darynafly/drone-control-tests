import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String, Header
import sys
import time
import numpy as np
from mavros_msgs.srv import CommandBool, SetMode
import math

class Setpointer(Node):

    def __init__(self, target_x, target_y, target_z, target_angle):
        super().__init__('set_goal')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.goal_reach_precision = 1.0
        self.angle_reach_precision = 0.5
        
        self.target_pos = (target_x, target_y, target_z)
        self.target_angle = math.radians(target_angle)  # Convert angle from degrees to radians

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
        if not self.is_at_position(x, y, z, self.goal_reach_precision, self.angle_reach_precision):
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            
            # Set orientation using the desired angle (yaw)
            quaternion = self.yaw_to_quaternion(self.target_angle)
            msg.pose.orientation = quaternion

            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Reached target position.')
            self.set_mode('AUTO.LOITER')

            #self.timer.cancel()  # Cancel the timer to stop publishing setpoints
            #rclpy.shutdown()     # Shut down the ROS node to stop everything

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

    def is_at_position(self, x, y, z, position_offset, angle_offset_degrees):
        """Check if the drone is at the given position and orientation."""
        desired_position = np.array((x, y, z))
        current_position = np.array((self.local_pos.pose.position.x,
                                    self.local_pos.pose.position.y,
                                    self.local_pos.pose.position.z))
        
        # Calculate position error
        position_error = np.linalg.norm(desired_position - current_position) < position_offset

        # Calculate current orientation (yaw) from quaternion
        orientation = self.local_pos.pose.orientation
        current_yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

        # Calculate angle error
        angle_error = abs(math.degrees(current_yaw) - self.target_angle) < angle_offset_degrees

        return position_error and angle_error

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 5:
        print("Usage: python3 move_to_xyz.py x y z angle")
        return

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_z = float(sys.argv[3])
    target_angle = float(sys.argv[4])

    set_goal = Setpointer(target_x, target_y, target_z, target_angle)
    try:
        rclpy.spin(set_goal)
    except KeyboardInterrupt:
        pass
    finally:
        set_goal.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
