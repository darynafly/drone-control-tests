from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class FollowObject(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.color = [0,0,0]
        self.object_center = None
        self.lower_hsv, self.upper_hsv = self.calculate_hsv_range(self.color)

        self.bridge = CvBridge()
        self.blue_object_center = None

        # Image subscriber
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Image publisher for contours
        self.contour_image_pub = self.create_publisher(
            Image,
            '/camera/contours',
            10
        )

    @staticmethod
    def calculate_hsv_range(color):
        # Convert the RGB color to an HSV color
        color_bgr = np.uint8([[color]])  # Convert to a format OpenCV expects
        color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]

        # Define a range around the HSV value for detection
        lower_hsv = np.array([max(0, color_hsv[0] - 10), 100, 100])
        upper_hsv = np.array([min(179, color_hsv[0] + 10), 255, 255])

        return lower_hsv, upper_hsv

    def detect_object(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Create a mask for the desired color
        mask = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                self.object_center = (cX, cY)
                print(f"Detected object at: {self.object_center}")

                # Draw the contour and center
                cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            else:
                self.object_center = None
        else:
            self.object_center = None

        # Publish the image with contours
        contour_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.contour_image_pub.publish(contour_image_msg)
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect_object(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    follower = FollowObject()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
