from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

class FollowObject(Node):
    def __init__(self):
        super().__init__('object_follower')

        self.x_vel = 1.5
        self.z_vel = 1.0
        self.yaw_vel = 0.5

        self.color = [0,0,0] # setup for red color
        self.object_center = None
        self.lower_hsv, self.upper_hsv = self.calculate_hsv_range(self.color)

        self.bridge = CvBridge()
        self.blue_object_center = None
        self.velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        self.angular = 0.0
        self.image_center = None
        self.current_position = [0.0, 0.0, 0.0]
        self.detected_pose = [0.0, 0.0]

        # Publishers
        self.position_target_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # Image publisher for contours
        self.contour_image_pub = self.create_publisher(
            Image,
            '/camera/contours',
            10
        )

        # Image subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # QoS settings for subscribers
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=qos_profile)

        self.get_logger().info("Object follower initialized.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.follow_object(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        #orientation_q = msg.pose.orientation
        #_, _, yaw = self.euler_from_quaternion(orientation_q)
        #self.current_yaw = yaw

    @staticmethod
    def calculate_hsv_range(color):
        # Convert the RGB color to an HSV color
        color_bgr = np.uint8([[color]])  # Convert to a format OpenCV expects
        color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]

        # Define a range around the HSV value for detection
        lower_hsv = np.array([max(0, color_hsv[0] - 10), 100, 100])
        upper_hsv = np.array([min(179, color_hsv[0] + 10), 255, 255])

        return lower_hsv, upper_hsv
    
    def follow_object(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define the blue color range in HSV
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                self.blue_object_center = (cX, cY)
                #self.get_logger().info(f"Detected blue object at: {self.blue_object_center}")

                # Draw the contour and center
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                x, y, w, h = cv2.boundingRect(largest_contour)  # (x, y) is the top-left corner, (w, h) are width and height
                self.detected_pose = [cX, cY]
                self.move([cX, cY], w, h )
                print(f"Detected object at: {self.object_center}, size: {w, h}")
            else:
                self.object_center = None
        else:
            self.object_center = None
            self.search(self.detected_pose)

        # Publish the image with contours
        contour_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.contour_image_pub.publish(contour_image_msg)

    def search(self, last_detected_pose):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ
        )

        yaw = self.yaw_vel

        if last_detected_pose[0] < 300:
            position_target.yaw = yaw
        else:
            position_target.yaw = -yaw
        if last_detected_pose[1] > 300:
            linear_z = -self.z_vel # Move down
        else:
            linear_z = self.z_vel # Move up
        
        if self.current_position[2] > 2.0:
            position_target.velocity.z = linear_z
        else:
            position_target.velocity.z = 0.2

        self.position_target_pub.publish(position_target)

    def move(self, detected_pose, width, height):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ
        )

        linear_x = 0.0
        linear_y = 0.0
        linear_z = 0.0 
        yaw = 0.0

        if detected_pose[0] < 300:
            yaw = self.yaw_vel 
        #    linear_y = vel
        elif detected_pose[0] > 350:
            yaw = -self.yaw_vel 
        #    linear_y = -vel

        if detected_pose[1] > 350:
            linear_z = -self.z_vel # Move down
        elif detected_pose[1] < 300:
            linear_z = self.z_vel # Move up

        area_threshold = 130
        if width < area_threshold - 20:
            linear_x = self.x_vel  # Move closer
        elif width > area_threshold:
            linear_x = -self.x_vel  # Move away
            
        position_target.velocity.x = linear_x
        position_target.velocity.y = linear_y
        position_target.yaw = yaw

        if self.current_position[2] > 2.0:
            position_target.velocity.z = linear_z
        else:
            position_target.velocity.z = 0.2

        self.position_target_pub.publish(position_target)

        #self.get_logger().info(f"Moving")

def main(args=None):
    rclpy.init(args=args)
    follower = FollowObject()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

