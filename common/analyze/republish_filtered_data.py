import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class MAVROSFilteredData(Node):
    def __init__(self):
        super().__init__('mavros_filtered_data')
        
        # Subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(NavSatFix, '/mavros/global_position/raw/fix', self.gps_callback, qos_profile)
        
        # filtered gps data 
        self.altitude_filtered_publisher = self.create_publisher(Float64, '/mavros/gps_altitude_filtered', 10)
        self.gps_filtered_publisher = self.create_publisher(Odometry, '/mavros/odom_altitude_filtered', 10)

        # Data storage
        self.init = False
        self.altitude_offset = None
        self.altitude_msg = Float64()

    def gps_callback(self, msg):
        #latitude = msg.latitude
        #longitude = msg.longitude
        if not self.init:
            self.altitude_offset = msg.altitude            
            self.init = True
        else:
            self.altitude_msg.data = msg.altitude - self.altitude_offset
            self.altitude_filtered_publisher.publish(self.altitude_msg)

            gps_msg = Odometry()
            gps_msg.header.frame_id = "map"
            gps_msg.pose.pose.position.z = msg.altitude - self.altitude_offset
            self.gps_filtered_publisher.publish(gps_msg)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MAVROSFilteredData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
