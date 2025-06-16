import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
import time

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_feedback')

        # Subscribers
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_callback, 10)
        
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

        # Ensure OFFBOARD mode is set and vehicle is armed
        self.offboard_mode_set = False
        self.arm_cmd_sent = False

        #self.set_mode('AUTO.LOITER')

        # Start arming process
        while not self.arm_call(True):
            self.get_logger().info("Attempting to arm the vehicle...")
            time.sleep(0.5)

        self.get_logger().info("Vehicle armed")

        self.target = PositionTarget()
        self.target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        self.target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        #| PositionTarget.IGNORE_YAW
        #| PositionTarget.IGNORE_YAW_RATE
        
        self.height = 0.3
        self.vel = 0.2

        self.target.header.frame_id = "map"
        self.target.yaw = 5.0
        #self.target.yaw_rate = 50.0

        # Publish initial setpoints for OFFBOARD mode activation
        for _ in range(100):  # Publish initial position setpoints
            self.local_target_pub.publish(self.target)
            time.sleep(0.05)
        
        # Set mode to OFFBOARD
        if self.set_mode('GUIDED'):
            self.get_logger().info(" mode enabled")
        else:
            self.get_logger().error("Failed to set  mode")

        self.timer = self.create_timer(0.01, self.timer_callback)  # 10Hz timer
        
    def state_callback(self, msg):
        self.current_state = msg

    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
    
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

    def timer_callback(self):
        self.target = PositionTarget()
        self.target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        self.target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        #| PositionTarget.IGNORE_YAW
        #| PositionTarget.IGNORE_YAW_RATE
        
        self.height = 0.3
        self.vel = 0.2

        self.target.header.frame_id = "map"
        #self.target.yaw = 0.5
        self.target.yaw_rate = 1.5
        self.local_target_pub.publish(self.target)  # Publish position setpoint

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