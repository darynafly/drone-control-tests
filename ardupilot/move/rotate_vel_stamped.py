import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
import math
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class SquarePathFollower(Node):
    def __init__(self):
        super().__init__('square_path_follower')

        # Declare parameters
        self.declare_parameter('linear_x_speed', 1.0)
        self.declare_parameter('angular_z_speed', 0.5)

        # Get parameters
        self.linear_x_speed = self.get_parameter('linear_x_speed').get_parameter_value().double_value
        self.angular_z_speed = self.get_parameter('angular_z_speed').get_parameter_value().double_value

        # Define QoS profiles for subscriber and publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,#QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE #QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE            
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=qos_profile)

        self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # Initialize variables
        self.current_yaw = 0.0
        self.move_stage = 0  # 0: forward, 1: turn right, 2: backward, 3: turn left

        # Timer to keep updating the movement
        self.timer = self.create_timer(0.01, self.move)

    def pose_callback(self, msg):
        orientation_q = msg.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.current_yaw = yaw

    def euler_from_quaternion(self, q):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def move_forward(self):
        #linear_x = self.linear_x_speed * math.cos(self.current_yaw)
        #linear_y = self.linear_x_speed * math.sin(self.current_yaw)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5 #linear_x
        cmd_vel.linear.y = 0.0 #linear_y
        cmd_vel.angular.z = 0.0
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Moving forward")

    def move_backward(self):
        #linear_x = -self.linear_x_speed * math.cos(self.current_yaw)
        #linear_y = -self.linear_x_speed * math.sin(self.current_yaw)

        cmd_vel = Twist()
        cmd_vel.linear.x = -1.0 #linear_x
        cmd_vel.linear.y = 0.0 #linear_y
        cmd_vel.angular.z = 0.0
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Moving backward")

    def move_left(self):
        #linear_x = self.linear_x_speed * math.cos(self.current_yaw)
        #linear_y = self.linear_x_speed * math.sin(self.current_yaw)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0 #linear_x
        cmd_vel.linear.y = 1.0 #linear_y
        cmd_vel.angular.z = 0.0
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Moving left")

    def move_right(self):
        #linear_x = -self.linear_x_speed * math.cos(self.current_yaw)
        #linear_y = -self.linear_x_speed * math.sin(self.current_yaw)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = -1.0 #linear_y
        cmd_vel.angular.z = 0.0
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Moving right")

    def turn_right(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = -self.angular_z_speed
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Turning right")

    def turn_left(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = self.angular_z_speed
        self.publisher.publish(cmd_vel)
        self.get_logger().info(f"Turning left")

    def move(self):
        cmd_vel = TwistStamped()
        cmd_vel.twist.linear.x = 0.0
        cmd_vel.twist.linear.y = 0.0
        cmd_vel.twist.linear.z = 0.0
        cmd_vel.twist.angular.x = 0.0
        cmd_vel.twist.angular.z = -5.0
        self.publisher.publish(cmd_vel)

    def move_(self):
        if self.move_stage == 0:
            self.move_forward()  # forward
            self.move_stage = 1 

        elif self.move_stage == 1:
            self.move_right()  # right
            self.move_stage = 2 

        elif self.move_stage == 2:
            self.move_backward()  # backward
            self.move_stage = 3  

        elif self.move_stage == 3:
            self.move_left()  # left
            self.move_stage = 0  # Reset 

def main(args=None):
    rclpy.init(args=args)
    square_path_follower = SquarePathFollower()
    rclpy.spin(square_path_follower)
    square_path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
