import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Twist
import time

class Setpointer(Node):

    def __init__(self):
        super().__init__('takeoff')

        # Create a QoS profile with BestEffort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # ------- client -------------#
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode setting service...')

        # self.set_mode_srv = self.create_client(CommandBool, '/mavros/cmd/arming',qos_profile=qos_profile)
        self.set_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.set_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Set target altitude and speed
        self.target_altitude = 1.0  # Desired altitude in meters
        self.takeoff_speed = 0.5  # Vertical speed in m/s
        
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
    
    def Main(self):
        armed = False
        while not armed:
            #self.set_mode('MANUAL')
            #self.set_mode('STABILIZED')    
            #self.set_mode('AUTO.TAKEOFF')
            #self.set_mode('OFFBOARD') 
            self.set_mode('AUTO.TAKEOFF')
            armed = self.arm_call(True)
            time.sleep(1)

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
        set_goal.set_mode('AUTO.LAND')
        set_goal.destroy_node()
        rclpy.shutdown()
    
   
    

if __name__ == '__main__':
    main()
