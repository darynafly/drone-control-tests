import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import AttitudeTarget
import math
import time

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_feedback')

        self.desired_x = 0.0
        self.desired_y = 0.0

        #limits for roll and pitch 
        self.roll_max = 0.2
        self.pitch_max = 0.2

        self.accuracy_threshold = 0.1

        # drone current xyz, rpy
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        # Subscribers
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_callback, 10)
        
        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)
        self.local_vel_pub = self.create_publisher(
            Twist, 'mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.pub_attitude = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_imu = self.create_subscription(
            Odometry, 
            '/estimated/imu_odometry', 
            self.odom_callback, 
            qos_profile
        )

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
        self.desired_takeoff_time = 4.0
        self.takeoff_thrust = 0.6

        self.offboard_mode_set = False
        self.arm_cmd_sent = False
        self.armed = False

        #AUTO.PRECLAND AUTO.FOLLOW_TARGET AUTO.TAKEOFF MANUAL AUTO.MISSION ACRO AUTO.LOITER ALTCTL AUTO.RTGS OFFBOARD POSCTL AUTO.LAND STABILIZED RATTITUDE AUTO.RTL AUTO.READY

        self.set_mode('GUIDED_NOGPS')
        
        self.get_logger().info("Vehicle arming...")
        self.arming(True)
            
        time.sleep(2.0)

        self.get_logger().info("Vehicle armed")
        self.armed = True

        self.timer = self.create_timer(0.1, self.timer_callback)  # 20Hz
    

    def state_callback(self, msg):
        self.current_state = msg

    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def arming(self, arm: bool = False):
        """Arm or disarm the drone

        Args:
            arm (bool, optional): `True` - arm, `False` - disarm.
            Defaults to False.
        """
        while not (arm and self.current_state.armed):
            if self.arming_client.wait_for_service(timeout_sec=1.0):
                arming_call = CommandBool.Request()
                arming_call.value = arm
                future = self.arming_client.call_async(arming_call)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().success:
                    self.get_logger().info('Success')
                    break
                self.get_logger().warn('Failed. Retrying...')
            time.sleep(0.5)
            
    
    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.z = odom_msg.pose.pose.position.z
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(odom_msg.pose.pose.orientation)
        print(self.x, self.y, self.z)

    def euler_from_quaternion(self, q):
        """
        Converts a quaternion to Euler angles (roll, pitch, yaw).
        
        Args:
        - q (Quaternion): The quaternion to convert.

        Returns:
        - Tuple[float, float, float]: The roll, pitch, and yaw angles in radians.
        """
        t0 = 2.0 * (q.w * q.x + q.y * q.z)
        t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (q.w * q.y - q.z * q.x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def timer_callback(self):
        if self.armed:
            if time.time() - self.time_start < self.desired_takeoff_time:
                msg = AttitudeTarget()
                msg.thrust = 0.55 if self.z > 0.5 else self.takeoff_thrust

                # Pitch control (forward/backward movement)
                pitch_error = self.desired_x-self.x  # Error between current position and desired
                pitch = max(-self.pitch_max, min(self.pitch_max, pitch_error * 0.2))  # Proportional control for pitch
                
                # Roll control (right/left movement)
                roll_error = self.desired_y-self.y  # Error between current position and desired
                roll = max(-self.roll_max, min(self.roll_max, roll_error * 0.2))  # Proportional control for roll

                yaw = self.yaw  # Maintain current yaw for stability
                
                msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                self.pub_attitude.publish(msg)

                print("takeoff")
    
            else:
                print("land")
                self.set_mode('LAND')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated by user.')
        #node.set_mode('GUIDED_NOGPS')
        #node.set_mode('LAND')
        node.arming(False)
    finally:
        #node.set_mode('LAND')
        #node.set_mode('GUIDED_NOGPS')
        node.arming(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
