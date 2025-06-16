import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math
import time

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_feedback')

        # Subscribers
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_callback, 10)
        
        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)
        self.local_vel_pub = self.create_publisher(
            Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Service Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.arming_client.wait_for_service()
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_mode_client.wait_for_service()

        # State and timer setup
        self.current_state = State()
        self.pose = PoseStamped()
        self.vel = Twist()
        
        self.pose.header.frame_id = "map"
        self.pose.pose.position.z = 2.0
        self.time_start = time.time()
        
        self.offboard_mode_set = False
        self.arm_cmd_sent = False

        #AUTO.PRECLAND AUTO.FOLLOW_TARGET AUTO.TAKEOFF MANUAL AUTO.MISSION ACRO AUTO.LOITER ALTCTL AUTO.RTGS OFFBOARD POSCTL AUTO.LAND STABILIZED RATTITUDE AUTO.RTL AUTO.READY

        while not self.arm_call(True):
            self.get_logger().info("Vehicle arming...")
            time.sleep(0.5)

        #self.set_mode('OFFBOARD')

        self.get_logger().info("Vehicle armed")

        self.timer = self.create_timer(0.1, self.timer_callback)  # 20Hz
        
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
        return arm_resp.success

    def timer_callback(self):
        print("test")


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated by user.')
        #node.set_mode('AUTO.LAND')
        node.arm_call(False)
    finally:
        #node.set_mode('AUTO.LAND')
        node.arm_call(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
