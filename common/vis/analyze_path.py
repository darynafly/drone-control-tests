import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
import numpy as np
import matplotlib.pyplot as plt
from rclpy.duration import Duration


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.height = 0.0
        self.last_gps_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=5.0)  # Timeout of 1 second

        self.waypoints = [
            [0.0, 0.0, self.height], 
            #[0.0, -7.0 , self.height],
            #[0.0, -7.0 , self.height],
            #[0.0, 0.0, self.height],
            [12.0, 0.0 , self.height], 
            [12.0, 0.0 , self.height],  
            #[12.0, -7.0 , self.height], 
            #[12.0, -7.0 , self.height], 
            #[0.0, -7.0 , self.height], 
            [0.0, 0.0, self.height],
        ]

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription_gps_odom = self.create_subscription(
            Odometry, 'gps_shifted_odometry', self.gps_odom_callback, qos_profile
        )
        self.subscription_imu_odom = self.create_subscription(
            Odometry, 'imu_shifted_odometry', self.imu_odom_callback, qos_profile
        )

        self.planned_path = np.array(self.waypoints)[:, :2]  # Ideal path (x, y)
        self.gps_path = []
        self.imu_path = []

        # Timer to check for timeouts
        self.timer = self.create_timer(0.1, self.check_timeout)

    def gps_odom_callback(self, msg):
        self.last_gps_time = self.get_clock().now()
        self.gps_path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def imu_odom_callback(self, msg):
        self.last_imu_time = self.get_clock().now()
        self.imu_path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def check_timeout(self):
        now = self.get_clock().now()
        gps_timeout = (now - self.last_gps_time) > self.timeout_duration
        imu_timeout = (now - self.last_imu_time) > self.timeout_duration

        if gps_timeout and imu_timeout:
            self.visualize_paths()
            rclpy.shutdown()


    def calculate_total_deviation(self, ideal_path_np, estimated_path_np):
        """
        Calculate the total deviation of the estimated path from the ideal path.
        Each point on the estimated path is compared with the closest point on the ideal path.
        
        Args:
        - ideal_path_np (numpy.ndarray): Nx2 array representing the ideal path (x, y points).
        - estimated_path_np (numpy.ndarray): Mx2 array representing the estimated path (x, y points).

        Returns:
        - total_deviation (float): Total deviation as the sum of minimal distances between the paths.
        """
        # Initialize total deviation
        total_deviation = 0.0

        # Iterate over each point in the estimated path
        for estimated_point in estimated_path_np:
            # Compute distances from the estimated point to all points in the ideal path
            distances = np.linalg.norm(ideal_path_np - estimated_point, axis=1)

            # Find the minimum distance to the ideal path
            min_distance = np.min(distances)

            # Add the minimum distance to the total deviation
            total_deviation += min_distance

        return total_deviation


    def visualize_paths(self):
        # Handle different types for paths (check if already NumPy arrays)
        if isinstance(self.planned_path, np.ndarray):
            planned_path_np = self.planned_path
        else:
            planned_path_np = np.array(self.planned_path)

        if isinstance(self.gps_path, np.ndarray):
            gps_path_np = self.gps_path
        else:
            gps_path_np = np.array(self.gps_path)

        if isinstance(self.imu_path, np.ndarray):
            imu_path_np = self.imu_path
        else:
            imu_path_np = np.array(self.imu_path)

        # Skip visualization if planned_path is empty
        if len(planned_path_np) == 0:
            self.get_logger().warning("No planned path data available for visualization.")
            return
        if len(gps_path_np) == 0:
            self.get_logger().warning("No GPS path data received.")
        if len(imu_path_np) == 0:
            self.get_logger().warning("No IMU path data received.")

        gps_deviations = self.calculate_total_deviation(planned_path_np, gps_path_np)
        print("GPS Path deviations (in meters):", gps_deviations)

        imu_deviations = self.calculate_total_deviation(planned_path_np, imu_path_np)
        print("IMU Path deviations (in meters):", imu_deviations)

        # Plot setup
        plt.figure(figsize=(18, 6))

        # Plot all paths together
        plt.subplot(1, 3, 1)
        plt.plot(planned_path_np[:, 0], planned_path_np[:, 1], 'g-', label="Ideal", linewidth=2)
        if len(gps_path_np) > 0:
            plt.plot(gps_path_np[:, 0], gps_path_np[:, 1], 'b-', label="GPS", linewidth=2)
        if len(imu_path_np) > 0:
            plt.plot(imu_path_np[:, 0], imu_path_np[:, 1], 'orange', label="IMU", linewidth=2)
        plt.title("All Paths")
        plt.legend()
        plt.axis('equal')

        # Plot IMU vs Ideal
        plt.subplot(1, 3, 2)
        plt.plot(planned_path_np[:, 0], planned_path_np[:, 1], 'g-', label="Ideal", linewidth=2)
        if len(imu_path_np) > 0:
            plt.plot(imu_path_np[:, 0], imu_path_np[:, 1], 'orange', label="IMU", linewidth=2)
        plt.title("IMU vs Ideal")
        plt.legend()
        plt.axis('equal')

        # Plot GPS vs Ideal
        plt.subplot(1, 3, 3)
        plt.plot(planned_path_np[:, 0], planned_path_np[:, 1], 'g-', label="Ideal", linewidth=2)
        if len(gps_path_np) > 0:
            plt.plot(gps_path_np[:, 0], gps_path_np[:, 1], 'b-', label="GPS", linewidth=2)
        plt.title("GPS vs Ideal")
        plt.legend()
        plt.axis('equal')

        # Show plot
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
