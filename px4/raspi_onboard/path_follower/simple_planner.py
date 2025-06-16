import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget, Altitude
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from mavros_msgs.srv import SetMode, CommandBool

from nav_msgs.msg import Path
from std_msgs.msg import Header

from path_accuracy_calculations import calculate_metrics, calculate_deviations, path_length_difference #, visualize_paths

import math
import numpy as np
import time

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.accuracy_threshold = 2.0

        # Desired speed
        self.desired_speed = 1.0  # m/s 1.8
        self.tolerance_2d = 1.0
        self.tolerance_z = 1.0

        self.takeoff_height = 3.0
        self.takeoff_vel = 1.0
        self.land_vel = 1.0
        self.current_altitude = 0.0
        
        rect_side = 13.0

        self.mode_auto_land = True
        self.mode_auto_takeoff = False

        path_mode = 1
        if path_mode == 0: #point
            self.waypoints = [
                [0.0, 0.0, self.takeoff_height],
            ]
        elif path_mode == 1: #line
            self.waypoints = [
                [0.0, 0.0, self.takeoff_height], 
                [rect_side, 0.0 , self.takeoff_height], 
                [0.0, 0.0, self.takeoff_height],
            ]
        elif path_mode == 2: #square
            self.waypoints = [
                # Test 1
                # big square different altitudes
                #
                #[0.0, -0.5, self.takeoff_height], 
                #[-6.0, -0.5, 2.5], 
                #[-6.0, -3.5, 1.0], 
                #[-6.0, -6.5, self.takeoff_height], 
                #[0.0, -6.5, 2.5], 
                #[0.0, -0.5, self.takeoff_height],     

                #[0.0, -0.5, self.takeoff_height], 
                #[-6.0, -0.5, 2.5], 
                #[-6.0, -3.5, 1.0], 
                #[-6.0, -6.5, self.takeoff_height], 
                #[0.0, -6.5, 2.5], 
                #[0.0, -0.5, self.takeoff_height],   

                #[0.0, -0.5, self.takeoff_height], 
                #[-6.0, -0.5, 2.5], 
                #[-6.0, -3.5, 1.0], 
                #[-6.0, -6.5, self.takeoff_height], 
                #[0.0, -6.5, 2.5], 
                #[0.0, -0.5, self.takeoff_height], 

                # small square same altitudes
                #
                #[0.0, -0.5, self.takeoff_height], 
                #[-4.0, -0.5,  self.takeoff_height], 
                #[-4.0, -4.5, self.takeoff_height], 
                #[0.0, -4.5,  self.takeoff_height], 
                #[0.0, -0.5, self.takeoff_height], 

                #[0.0, -0.5, self.takeoff_height], 
                #[-4.0, -0.5,  self.takeoff_height], 
                #[-4.0, -4.5, self.takeoff_height], 
                #[0.0, -4.5,  self.takeoff_height], 
                #[0.0, -0.5, self.takeoff_height], 

                
                # Test 2 - Three same altitudes squares
                #
                #[0.0, -0.5, self.takeoff_height], 
                #[-5.0, -0.5,  self.takeoff_height], 
                #[-5.0, -5.5, self.takeoff_height], 
                #[0.0, -5.5,  self.takeoff_height], 
                #[0.0, -0.5, self.takeoff_height], 

                #[0.0, -0.5, self.takeoff_height], 
                #[-5.0, -0.5,  self.takeoff_height], 
                #[-5.0, -5.5, self.takeoff_height], 
                #[0.0, -5.5,  self.takeoff_height], 
                #[0.0, -0.5, self.takeoff_height], 

                #[0.0, -0.5, self.takeoff_height], 
                #[-5.0, -0.5,  self.takeoff_height], 
                #[-5.0, -5.5, self.takeoff_height], 
                #[0.0, -5.5,  self.takeoff_height], 
                #[0.0, -0.5, self.takeoff_height], 

                # Test 3 - Line 
                #
                #[0.0, -0.5, self.takeoff_height], 
                #[-30.0, -0.5,  self.takeoff_height], 
                #[0.0, -0.5, self.takeoff_height], 
        ]
        self.update_rate = 10.

        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=qos_profile
        )
        
        self.alt_sub = self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.altitude_cb,
            qos_profile=qos_profile
        )

        # Publisher
        self.position_target_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # Clients for services
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

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
        self.path.header.frame_id = "map"  # Set the frame ID for the path
        self.path_all = Path()
        self.path_all.header.frame_id = "map"
        self.path_current_real = Path()
        self.path_current_real.header.frame_id = "map"

        self.np_path_all = np.empty((0, 3))
        self.np_path_real = np.empty((0, 3))
        
        self.waypoints_reached = False
        self.reached_first = False

        # Add a IDEAL waypoints to the path
        for waypoint in self.waypoints:    
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation

            self.path_all.poses.append(pose)  # Add the pose to the path
            self.path_all.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp

        #if self.mode_auto_takeoff:
        #    self.auto_takeoff()
        #else:
        self.velocity_takeoff()

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.navigate)

    def visualise_plan_path(self):
        # Add a new waypoint to the path
        if self.target_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.target_waypoint_index]
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
            pose.header.frame_id = "map"
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation

            self.path.poses.append(pose)  # Add the pose to the path
            self.path.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp
            self.publisher_path.publish(self.path)  # Publish the path
        else:
            self.publisher_path.publish(self.path)  # Publish the path


    def visualise_path_all(self):
        #self.np_path_all = np.vstack((self.np_path_all, np.array([waypoint[0], waypoint[1], waypoint[2]])))
        self.publisher_path_all.publish(self.path_all)  # Publish the path
            
    def visualise_path_current_real(self, waypoint):
        # Add a new waypoint to the path
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()  # Timestamp
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  # Default orientation

        self.path_current_real.header.stamp = self.get_clock().now().to_msg()  # Update the path timestamp
        self.publisher_path_curr.publish(self.path_current_real)  # Publish the path

        if not self.waypoints_reached and self.reached_first:
            self.path_current_real.poses.append(pose)  # Add the pose to the path


    def altitude_cb(self, msg):
        """Checks if the drone has reached take off or landing height"""
        self.current_altitude = (
            msg.relative
        )  # relative to home

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orientation_q = msg.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)
        self.visualise_path_current_real(self.current_position)

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

    def navigate(self):
        if self.target_waypoint_index >= len(self.waypoints):
            self.waypoints_reached = True

            if self.mode_auto_land:
                self.auto_land()
            else:
                self.velocity_land()

            # Calculate the metrics
            accuracy, precision, recall, f1_score = calculate_metrics(self.path_all, self.path_current_real, self.accuracy_threshold)
            mean_deviation, min_deviation, max_deviation, deviations = calculate_deviations(self.path_all, self.path_current_real)
            path_diff = path_length_difference(self.path_all, self.path_current_real)

            print(f"All waypoints reached.")
            print(f"Threshold {self.accuracy_threshold}")

            print(f"Length difference {path_diff} m")       
            print(f"Average Deviation: {mean_deviation:.4f} m")
            print(f"Maximum Deviation: {max_deviation:.4f} m")
            print(f"Minimum Deviation: {min_deviation:.4f} m")
            print(f"Correctness: {f1_score:.2f}%")

            #visualize_paths(self.path_all, self.path_current_real, self.accuracy_threshold)
          
            return

        target_waypoint = self.waypoints[self.target_waypoint_index]
        distance_to_target_2d = math.sqrt(
            (target_waypoint[0] - self.current_position[0]) ** 2 +
            (target_waypoint[1] - self.current_position[1]) ** 2
        )

        # Check if the distance to the waypoint is within tolerance in 2D and z
        if distance_to_target_2d < self.tolerance_2d and abs(target_waypoint[2] - self.current_altitude) < self.tolerance_z:
            self.get_logger().info(f"Reached waypoint {target_waypoint}.")
            self.stop()    
            if self.target_waypoint_index == 0:
                self.reached_first = True
            self.target_waypoint_index += 1
        else:
            self.publish_setpoint(target_waypoint)
        self.visualise_plan_path()
        self.visualise_path_all()

    def stop(self):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )

        # Set velocity to stop
        position_target.velocity.x = 0.0
        position_target.velocity.y = 0.0
        position_target.velocity.z = 0.0 

        # Publish the setpoint
        self.position_target_pub.publish(position_target)

    def set_mode(self, mode):
        """Set the flight mode of the drone."""
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent

    def arm_drone(self, value):
        """Arm the drone."""
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        req = CommandBool.Request()
        req.value = value
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def auto_takeoff(self):
        armed = False
        while not armed:
            self.set_mode('AUTO.TAKEOFF')  # Ensure the drone is stabilized
            armed = self.arm_drone(True)
            self.get_logger().info("Taking off...")
            time.sleep(1)
        
        time.sleep(5.0)

        self.get_logger().info("Takeoff completed. Switching to OFFBOARD mode.")
        
        self.hover()

    def hover(self):
        """Take off to the specified altitude at the given speed and hover."""
        
        self.set_mode('AUTO.LOITER')

        self.get_logger().info("Arming the drone...")
        if not self.arm_drone(True):
            self.get_logger().error("Failed to arm the drone")
            return

        self.get_logger().info("Setting mode to OFFBOARD...")
        if not self.set_mode("OFFBOARD"):
            self.get_logger().error("Failed to set mode to OFFBOARD")
            return

        self.get_logger().info("Taking off...")
        rate = self.create_rate(20)  # 20 Hz

        while not self.current_position:
            self.get_logger().info("Waiting for pose data...")
            time.sleep(1.0)

        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX 
            | PositionTarget.IGNORE_PY 
            | PositionTarget.IGNORE_PZ 
            | PositionTarget.IGNORE_AFX 
            | PositionTarget.IGNORE_AFY 
            | PositionTarget.IGNORE_AFZ 
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE)
        target.velocity.z = 0.0

        self.position_target_pub.publish(target)
        rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Takeoff complete. Hovering at {:.2f} meters.".format(self.takeoff_height))


    def prearm(self):
        """Set take off velocity and publish points for OFFBOARD mode"""
        # Take off velocity
        
        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX 
            | PositionTarget.IGNORE_PY 
            | PositionTarget.IGNORE_PZ 
            | PositionTarget.IGNORE_AFX 
            | PositionTarget.IGNORE_AFY 
            | PositionTarget.IGNORE_AFZ 
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE)
        target.velocity.z = self.takeoff_vel

        self.get_logger().info("Prearm. Setting points...")
        # Send a few setpoints before starting OFFBOARD mode
        for _ in range(100):
            self.position_target_pub.publish(target)
            time.sleep(0.01)

    def velocity_takeoff(self):
        """Take off to the specified altitude at the given speed and hover."""
        
        #self.set_mode('AUTO.LOITER')

        self.prearm()

        self.get_logger().info("Setting mode to OFFBOARD...")
        if not self.set_mode("OFFBOARD"):
            self.get_logger().error("Failed to set mode to OFFBOARD")
            return

        self.get_logger().info("Arming the drone...")
        if not self.arm_drone(True):
            self.get_logger().error("Failed to arm the drone")
            return

        self.get_logger().info("Taking off...")
        rate = self.create_rate(20)  # 20 Hz

        while not self.current_position:
            self.get_logger().info("Waiting for pose data...")
            time.sleep(1.0)

        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX 
            | PositionTarget.IGNORE_PY 
            | PositionTarget.IGNORE_PZ 
            | PositionTarget.IGNORE_AFX 
            | PositionTarget.IGNORE_AFY 
            | PositionTarget.IGNORE_AFZ 
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE)
        target.velocity.z = self.takeoff_vel

        while self.current_altitude < self.takeoff_height - 0.1:  # Allow a small buffer
            target.velocity.z = self.takeoff_vel
            self.position_target_pub.publish(target)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Takeoff complete. Hovering at {:.2f} meters.".format(self.takeoff_height))

    def auto_land(self):
        self.get_logger().info("Initiating auto landing.")
        while not self.set_mode('AUTO.LAND'):
            print("Landing...")
            time.sleep(1)
        self.arm_drone(False)
        time.sleep(1)
        #self.stop()  # Final stop to ensure no residual movement
        self.get_logger().info("Landing completed.")

    def velocity_land(self):
        self.get_logger().info("Initiating controlled landing.")
        while self.current_altitude > 0.8:  # Ensure UAV lands gently
            target = PositionTarget()
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            target.type_mask = (
            PositionTarget.IGNORE_PX 
            | PositionTarget.IGNORE_PY 
            | PositionTarget.IGNORE_AFX 
            | PositionTarget.IGNORE_AFY 
            | PositionTarget.IGNORE_AFZ 
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE)
            target.velocity.x = 0.0
            target.velocity.y = 0.0
            target.velocity.z = -self.land_vel  # Controlled descent speed
            self.position_target_pub.publish(target)
            #self.visualise_path_current_real(self.current_position)
            rclpy.spin_once(self, timeout_sec=0.05)
            print("Slow landing")
            time.sleep(0.01)
            #self.get_clock().sleep_for(1.0 / self.update_rate)
        self.set_mode('AUTO.LAND')
        time.sleep(1)
        self.arm_drone(False)
        time.sleep(1)
        #self.stop()  # Final stop to ensure no residual movement
        self.get_logger().info("Landing completed.")

    def rotate(self, yaw_rate):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW
        )

        # Set velocity to move toward the waypoint
        position_target.velocity.x = 0.0
        position_target.velocity.y = 0.0
        position_target.velocity.z = 0.0

        # Calculate yaw to face the target
        position_target.yaw_rate = yaw_rate

        # Publish the setpoint
        self.position_target_pub.publish(position_target)
        #self.get_logger().info(f"Publishing velocity setpoint: vx={velocity_x:.2f}, vy={velocity_y:.2f}, vz={position_target.velocity.z:.2f}, yaw={yaw:.2f}")


    def publish_setpoint(self, target_waypoint):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW
        )

        # Calculate direction to waypoint
        direction_x = target_waypoint[0] - self.current_position[0]
        direction_y = target_waypoint[1] - self.current_position[1]
        direction_z = target_waypoint[2] - self.current_altitude
        distance_3d = math.sqrt(direction_x ** 2 + direction_y ** 2 + direction_z ** 2)
        distance_3d += 0.001

        if distance_3d > 0:
            velocity_x = (direction_x / distance_3d) * self.desired_speed
            velocity_y = (direction_y / distance_3d) * self.desired_speed
            velocity_z = (direction_z / distance_3d) * self.desired_speed
        else:
            velocity_x = 0.0
            velocity_y = 0.0
            velocity_z = 0.0

        # Set velocity to move toward the waypoint
        position_target.velocity.x = velocity_x
        position_target.velocity.y = velocity_y
        position_target.velocity.z = velocity_z * 0.8

        # Calculate desired yaw and yaw rate
        desired_yaw = math.atan2(direction_y, direction_x)
        yaw_error = desired_yaw - self.current_yaw

        # Normalize yaw error to the range [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        if abs(yaw_error) > 0.2:  # Adjust yaw rate only if error is significant
            max_yaw_rate = 1.3  # Adjust this value based on drone's capabilities
            position_target.yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, yaw_error * 2.0))  # Proportional control
        else:
            position_target.yaw_rate = 0.0

        # Publish the setpoint
        self.position_target_pub.publish(position_target)
        #self.get_logger().info(f"Publishing velocity setpoint: vx={velocity_x:.2f}, vy={velocity_y:.2f}, vz={position_target.velocity.z:.2f}, yaw={yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    set_goal = WaypointFollower()
    try:
        rclpy.spin(set_goal)
    except KeyboardInterrupt:
        set_goal.set_mode('AUTO.LAND')
        set_goal.arm_drone(False)
        set_goal.set_mode('STABILIZED')
    finally:
        set_goal.set_mode('AUTO.LAND')
        set_goal.arm_drone(False)
        set_goal.set_mode('STABILIZED')
        # Cleanup code
        set_goal.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
