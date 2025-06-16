import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import csv
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class MAVROSVisualizer(Node):
    def __init__(self):
        super().__init__('mavros_visualizer')
        
        # CSV file setup
        self.csv_filename = "gps_data.csv"
        file_exists = os.path.isfile(self.csv_filename)
        self.csv_file = open(self.csv_filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if not file_exists:
            #self.csv_writer.writerow(["timestamp", "rel_alt", "local_z", "gps_z_filtered"])
            self.csv_writer.writerow(["rel_alt", "local_z", "gps_z_filtered"])

        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(PositionTarget, '/mavros/setpoint_raw/local', self.position_target_callback, qos_profile)
        self.create_subscription(Odometry, '/mavros/local_position/odom', self.odometry_callback, qos_profile)
        self.create_subscription(Odometry, '/estimated/imu_odometry', self.imu_odometry_callback, qos_profile)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_callback, qos_profile)
        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self.gps_callback, qos_profile)
        
        # Data storage
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
        self.rel_alt = 0.0
        self.altitude_filtered = None
        self.altitude_offset = None
        self.init = False

        self.timer = self.create_timer(0.1, self.update_plot)  # 10Hz update rate

    def position_target_callback(self, msg: PositionTarget):
        self.vel_x = msg.velocity.x
        self.vel_y = msg.velocity.y
        self.vel_z = msg.velocity.z

    def odometry_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def imu_odometry_callback(self, msg: Odometry):
        self.imu_x = msg.pose.pose.position.x
        self.imu_y = msg.pose.pose.position.y
        self.imu_z = msg.pose.pose.position.z

    def rel_alt_callback(self, msg: Float64):
        self.rel_alt = msg.data

    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        if self.altitude_filtered is None:
            self.altitude_offset = msg.altitude
            self.altitude_filtered = 0.0
            self.init = True
        else:
            self.altitude_filtered = msg.altitude - self.altitude_offset

    def update_plot(self):
        if self.init:
            print(f"rel_alt: {self.rel_alt}, odom_z: {self.z}, gps_z: {self.altitude_filtered}")
        
            # Save data to CSV
            #self.csv_writer.writerow([self.get_clock().now().to_msg().sec, self.rel_alt, self.z, self.altitude_filtered])
            self.csv_writer.writerow([self.rel_alt, self.z, self.altitude_filtered])
            self.csv_file.flush()
            
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSVisualizer()
    try:
        plt.ion()  # Interactive mode on
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
