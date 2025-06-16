import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import subprocess
import os
import signal
import time
import re


class GazeboToROSPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_to_ros_publisher')
        # Publishers
        #self.pose_publisher = self.create_publisher(PoseStamped, '/mavros/mocap/pose', 10)
        #self.odom_publisher = self.create_publisher(Odometry, '/mavros/odometry/out', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/gazebo/odometry', 10)

        # Start the Gazebo command
        self.command = "gz topic -e -t /world/default/dynamic_pose/info"
        self.process = subprocess.Popen(
            self.command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )

        # Create a timer to process output
        self.timer = self.create_timer(0.1, self.process_gazebo_output)

        # Flags and temporary storage
        self.flags = self.reset_flags()
        self.position = {}
        self.orientation = {}

    def reset_flags(self):
        return {
            "model_found": False,
            "position": False,
            "x_found": False,
            "y_found": False,
            "z_found": False,
            "orientation": False,
            "or_x_found": False,
            "or_y_found": False,
            "or_z_found": False,
            "or_w_found": False,
        }

    def process_gazebo_output(self):
        try:
            for line in iter(self.process.stdout.readline, ''):
                line = line.strip()

                if "x500_0" in line:
                    self.flags = self.reset_flags()
                    self.flags["model_found"] = True
                    #self.get_logger().info("Model Found: x500_0")

                if self.flags["model_found"]:
                    if "position" in line and not self.flags["position"]:
                        self.flags["position"] = True
                        self.flags["orientation"] = False
                        self.position = {}
                    elif "x:" in line and self.flags["position"] and not self.flags["x_found"]:
                        self.position['x'] = float(line.split(":")[1])
                        self.flags["x_found"] = True
                    elif "y:" in line and self.flags["position"] and not self.flags["y_found"]:
                        self.position['y'] = float(line.split(":")[1])
                        self.flags["y_found"] = True
                    elif "z:" in line and self.flags["position"] and not self.flags["z_found"]:
                        self.position['z'] = float(line.split(":")[1])
                        self.flags["z_found"] = True

                    if "orientation" in line and not self.flags["orientation"]:
                        self.flags["position"] = False
                        self.flags["orientation"] = True
                        self.orientation = {}
                    elif "x:" in line and self.flags["orientation"] and not self.flags["or_x_found"]:
                        self.orientation['x'] = float(line.split(":")[1])
                        self.flags["or_x_found"] = True
                    elif "y:" in line and self.flags["orientation"] and not self.flags["or_y_found"]:
                        self.orientation['y'] = float(line.split(":")[1])
                        self.flags["or_y_found"] = True
                    elif "z:" in line and self.flags["orientation"] and not self.flags["or_z_found"]:
                        self.orientation['z'] = float(line.split(":")[1])
                        self.flags["or_z_found"] = True
                    elif "w:" in line and self.flags["orientation"] and not self.flags["or_w_found"]:
                        self.orientation['w'] = float(line.split(":")[1])
                        self.flags["or_w_found"] = True
                        self.publish_to_ros_topics()
        except Exception as e:
            #self.get_logger().error(f"Error processing output: {e}")
            pass

    def publish_to_ros_topics(self):
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.position['x']
        pose_msg.pose.position.y = self.position['y']
        pose_msg.pose.position.z = self.position['z']
        pose_msg.pose.orientation.x = self.orientation['x']
        pose_msg.pose.orientation.y = self.orientation['y']
        pose_msg.pose.orientation.z = self.orientation['z']
        pose_msg.pose.orientation.w = self.orientation['w']

        # Publish PoseStamped
        #self.pose_publisher.publish(pose_msg)
        #self.get_logger().info(f"Published PoseStamped: {pose_msg}")

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose_msg.pose

        # Publish Odometry
        self.odom_publisher.publish(odom_msg)
        #self.get_logger().info(f"Published Odometry: {odom_msg}")

    def destroy_node(self):
        super().destroy_node()
        os.kill(self.process.pid, signal.SIGTERM)
        self.process.wait()
        self.get_logger().info("Gazebo process terminated.")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboToROSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
