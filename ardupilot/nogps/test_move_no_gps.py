import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

import time 
#from path_accuracy import compare_paths, path_length_difference, visualize_paths, calculate_metrics
from path_accuracy_calculations import calculate_metrics, calculate_deviations, path_length_difference, visualize_paths

import math
import numpy as np

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # Desired speed
        self.desired_speed = 1.5  # m/s
        self.tolerance_2d = 1.0
        self.tolerance_z = 1.0

        square_side = 6.0
        #square_side = 3.0

        self.waypoints = [
            [0.0, 0.0, 4.0], 
            [0.0, square_side, 4.0], 
            [square_side, square_side, 4.0],
            [square_side, 0.0, 4.0],
            [0.0, 0.0, 4.0]
        ]
        
        self.update_rate = 10.


        # Internal state
        self.current_position = [0.0, 0.0, 1.0]
        self.current_yaw = 0.0
        self.target_waypoint_index = 0

        # Publisher for the Path topic
        self.publisher_path = self.create_publisher(Path, 'planned_path_next', 10)
        self.publisher_path_all = self.create_publisher(Path, 'planned_path', 10)
        self.publisher_path_curr = self.create_publisher(Path, 'travelled_path', 10)
        self.current_index = 0  # Start at the first waypoint
        self.path = Path()
        self.path.header.frame_id = "odom"  # Set the frame ID for the path
        self.path_all = Path()
        self.path_all.header.frame_id = "odom"
        self.path_current_real = Path()
        self.path_current_real.header.frame_id = "odom"

        self.np_path_all = np.empty((0, 3))
        self.np_path_real = np.empty((0, 3))
        
        self.waypoints_reached = False

        # Add a IDEAL waypoints to the path
        for waypoint in self.waypoints:    
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation

            self.path_all.poses.append(pose)  # Add the pose to the path
            self.path_all.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp


        # for takeoff
        # Service Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.arming_client.wait_for_service()
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_mode_client.wait_for_service()

        self.time_start = time.time()
        self.desired_takeoff_time = 9.0
        self.desired_forward_time = 3.0
        self.desired_backward_time = 5.0
        self.pub_attitude = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.armed = False

        #AUTO.PRECLAND AUTO.FOLLOW_TARGET AUTO.TAKEOFF MANUAL AUTO.MISSION ACRO AUTO.LOITER ALTCTL AUTO.RTGS OFFBOARD POSCTL AUTO.LAND STABILIZED RATTITUDE AUTO.RTL AUTO.READY

        self.set_mode('GUIDED_NOGPS')

        while not self.arm_call(True):
            self.get_logger().info("Vehicle arming...")
            time.sleep(0.5)

        self.get_logger().info("Vehicle armed")
        self.armed = True
        self.takeoff_completed = False
        self.forward_completed = False

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.navigate)

    
    def arm_call(self, val):
        req = CommandBool.Request()
        req.value = val
        self.get_logger().info('arm: "%s"' % val)
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        arm_resp = future.result()
        if arm_resp is not None:
            self.get_logger().info('Arming response: %s' % arm_resp)
        else:
            self.get_logger().error('Failed to receive arm response')
        return arm_resp


    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def euler_from_quaternion(self, q):
        """Convert a quaternion to euler."""
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def navigate(self):
        if self.armed and not self.takeoff_completed:
            if time.time() - self.time_start < self.desired_takeoff_time:
                msg = AttitudeTarget()
                msg.thrust = 0.75 # Maintain a stable hover

                # Roll, pitch, and yaw
                roll = 0.0  # Positive value moves right; negative moves left
                pitch = 0.0  # Keep the pitch neutral to avoid forward/backward movement
                yaw = 0.0  # Maintain current yaw

                # Convert to quaternion
                msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                self.pub_attitude.publish(msg)

                print("takeoff")
                self.time_forward = time.time()
            else:
                print("takeoff completed")
                self.takeoff_completed = True

        if self.takeoff_completed:
            if time.time() - self.time_forward < self.desired_forward_time:
                print("move forward")
                msg = AttitudeTarget()
                msg.thrust = 0.75 # Maintain a stable hover

                # Roll, pitch, and yaw
                roll = 0.0  # Positive value moves right; negative moves left
                pitch = 0.1  # forward movement
                yaw = 0.0  # Maintain current yaw

                # Convert to quaternion
                msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                self.pub_attitude.publish(msg)

                self.time_backward = time.time()
            else:
                self.forward_completed = True
            if self.forward_completed:
                if time.time() - self.time_backward < self.desired_backward_time:
                    print("move backward")
                    msg = AttitudeTarget()
                    msg.thrust = 0.75 # Maintain a stable hover

                    # Roll, pitch, and yaw
                    roll = 0.0  # Positive value moves right; negative moves left
                    pitch = -0.2  # forward movement
                    yaw = 0.0  # Maintain current yaw

                    # Convert to quaternion
                    msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
                    self.pub_attitude.publish(msg)
                else:
                    self.stop()
            

    def stop(self):
        print("landing")
        msg = AttitudeTarget()
        msg.thrust = 0.0

        # Roll, pitch, and yaw
        roll = 0.0  # Positive value moves right; negative moves left
        pitch = 0.0  # forward movement
        yaw = 0.0  # Maintain current yaw

        # Convert to quaternion
        msg.orientation = self.euler_to_quaternion(roll, pitch, yaw)
        self.pub_attitude.publish(msg)

   

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
