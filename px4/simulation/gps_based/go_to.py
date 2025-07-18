import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import time
import numpy as np

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_feedback')

        self.waypoint = [
            (0.0, 0.0, 2.0),
            (5.0, 0.0, 2.0),
            (5.0, 5.0, 2.0),
            (0.0, 5.0, 2.0),
            (0.0, 0.0, 2.0)
        ]
        self.target_index = 0
        self.target_reached_presicion = 3.0
        self.takeoff_height = 0.5
        self.vel = 2.5

        self.stay_on_target = 0
        self.delay_counter = 0

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_callback, 10)
        
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        self.local_pos = PoseStamped()

        # Publishers        
        self.local_target_pub = self.create_publisher(
            PositionTarget, 'mavros/setpoint_raw/local', 10
        )
        # Service Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.arming_client.wait_for_service()
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_mode_client.wait_for_service()

        # State and timer setup
        self.current_state = State()
        self.target = PositionTarget()
        self.target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target.type_mask = (
            PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        #PositionTarget.IGNORE_PX
        #| PositionTarget.IGNORE_PY
        #| PositionTarget.IGNORE_PZ
        #| PositionTarget.IGNORE_VX
        #| PositionTarget.IGNORE_VY
        #| PositionTarget.IGNORE_VZ
        #| PositionTarget.IGNORE_AFX
        #| PositionTarget.IGNORE_AFY
        #| PositionTarget.IGNORE_AFZ
        #| PositionTarget.IGNORE_YAW
        #| PositionTarget.IGNORE_YAW_RATE

        self.target.header.frame_id = "map"
        self.target.position.z = self.takeoff_height  # Desired altitude for takeoff
        self.target.velocity.z = self.vel  # Desired velocity for takeoff
        
        # Ensure OFFBOARD mode is set and vehicle is armed
        self.offboard_mode_set = False
        self.arm_cmd_sent = False

        self.set_mode('AUTO.LOITER')

        # Start arming process
        while not self.arm_call(True):
            self.get_logger().info("Attempting to arm the vehicle...")
            time.sleep(0.5)

        self.get_logger().info("Vehicle armed")

        # Publish initial setpoints for OFFBOARD mode activation
        for _ in range(100):  # Publish initial position setpoints
            self.local_target_pub.publish(self.target)
            time.sleep(0.05)
        
        # Set mode to OFFBOARD
        if self.set_mode('OFFBOARD'):
            self.get_logger().info("OFFBOARD mode enabled")
        else:
            self.get_logger().error("Failed to set OFFBOARD mode")

        self.timer = self.create_timer(0.01, self.timer_callback)  # 10Hz timer

    def pose_callback(self, msg):
        self.local_pos.pose.position.x = msg.pose.position.x
        self.local_pos.pose.position.y = msg.pose.position.y
        self.local_pos.pose.position.z = msg.pose.position.z

    def state_callback(self, msg):
        self.current_state = msg

    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def arm_call(self, val):
        req = CommandBool.Request()
        req.value = val
        self.get_logger().info('arm: "%s"' % val)
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        arm_resp = future.result()
        if arm_resp is not None:
            self.get_logger().info('Arming response: %s' % arm_resp.success)
        else:
            self.get_logger().error('Failed to receive arm response')
        return arm_resp

    def target_reached(self, target):
        """Check if the drone is at the given position."""
        desired = np.array((target[0], target[1], target[2])) # x y z
        pos = np.array((self.local_pos.pose.position.x,
                        self.local_pos.pose.position.y,
                        self.local_pos.pose.position.z))
        print(f'{target}: {np.linalg.norm(desired - pos)}')
        return np.linalg.norm(desired - pos) < self.target_reached_presicion
    
    def timer_callback(self):
        self.move_to()

    def move_to(self):
        if self.target_index < len(self.waypoint):
            self.target.position.x = self.waypoint[self.target_index][0]
            self.target.position.y = self.waypoint[self.target_index][1]
            self.target.position.z = self.waypoint[self.target_index][2]
            self.target.velocity.x = self.vel
            self.target.velocity.y = self.vel
            self.target.velocity.z = self.vel
            self.target.type_mask = (
                PositionTarget.IGNORE_YAW
                | PositionTarget.IGNORE_YAW_RATE
            )
            if not self.target_reached(self.waypoint[self.target_index]):
                self.local_target_pub.publish(self.target)  # Publish position setpoint
            else:
                # Stay in one position for a bit
                self.local_target_pub.publish(self.target)
                self.stay_on_target += 1
                if self.stay_on_target > self.delay_counter:
                    self.target_index += 1
                    self.stay_on_target = 0
        else:
            self.set_mode('AUTO.LAND')
            self.arm_call(False)
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated by user.')
        node.set_mode('AUTO.LAND')
        node.arm_call(False)
    finally:        
        node.set_mode('AUTO.LAND')
        node.arm_call(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
