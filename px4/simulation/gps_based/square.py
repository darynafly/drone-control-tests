import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped,Quaternion
from std_msgs.msg import String , Header

from geometry_msgs.msg import Vector3
from mavros_msgs.msg import Waypoint , State
from mavros_msgs.srv import CommandBool , CommandTOLLocal , SetMode,WaypointPush
import time
import math
import numpy as np
class Setpointer(Node):

    def __init__(self):
        super().__init__('set_goal')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        #------- pub -------------#
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile)
        #Initialize target position
        self.target_pos = None
        self.position_index = 0
        # Define positions as a list of tuples
        self.positions = [
            (0.0, 0.0, 1.0),
            (15.0, 0.0, 1.0),
            (15.0, 15.0, 1.0),
            (0.0, 15.0, 1.0),
            (0.0, 0.0, 1.0)
        ]
        self.waypont_alt = 1.0

        self.precision = 0.5

        # Timer for publishing positions
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ------- sub -------------#
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.listener_callback,
            qos_profile)
        self.local_pos=PoseStamped()
        
        # ------- client -------------#
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        # self.set_mode_srv = self.create_client(CommandBool, '/mavros/cmd/arming',qos_profile=qos_profile)
        self.set_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.set_way = self.create_client(WaypointPush, '/mavros/mission/push')
        while not self.set_way.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        

        # ----------Main-------------#
        self.Main()
    
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
        return arm_resp
    
    def set_next_target(self):
        if self.position_index < len(self.positions):
            self.target_pos = self.positions[self.position_index]
            self.get_logger().info(f'Setting next target position: {self.target_pos}')
            self.position_index += 1
        else:
            self.get_logger().info('All target positions reached.')
            self.target_pos = None
            self.set_mode('AUTO.LAND')

    def timer_callback(self):
        # for OFFBOARD MODE, we must send to position order before mode change
        if self.target_pos is None:
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = self.waypont_alt
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0 
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
            return
        
        x, y, z = self.target_pos
        if not self.is_at_position(x, y, z, self.precision):
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0 
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('Reached target position.')
            self.set_next_target()

    def listener_callback(self, msg):
        self.local_pos = PoseStamped()
        self.local_pos.pose.position.x=msg.pose.position.x
        self.local_pos.pose.position.y=msg.pose.position.y
        self.local_pos.pose.position.z=msg.pose.position.z

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        desired = np.array((x, y, z))
        pos = np.array((self.local_pos.pose.position.x,
                        self.local_pos.pose.position.y,
                        self.local_pos.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def target_position(self,x,y,z):
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.get_logger().info(f'Setting target position: ({x}, {y}, {z})')
        while not self.is_at_position(x, y, z, self.precision):
            msg = PoseStamped()
            msg.header = Header()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
            self.publisher_.publish(msg)
            
        self.get_logger().info('Reached target position.')

    def Main(self):
        self.set_mode('AUTO.TAKEOFF')
        time.sleep(5)
        self.arm_call(True)
        self.set_mode('OFFBOARD')
        time.sleep(1)
        self.set_next_target()
        # self.set_way_func()
def main(args=None):
    rclpy.init(args=args)
    set_goal = Setpointer()
    try:
        rclpy.spin(set_goal)
    except KeyboardInterrupt:
        set_goal.set_mode('AUTO.LAND')
        pass
    finally:
        # Cleanup code
        set_goal.destroy_node()
        rclpy.shutdown()
    
   
    

if __name__ == '__main__':
    main()