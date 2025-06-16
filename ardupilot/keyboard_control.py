import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
import keyboard

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        # Desired velocity
        self.velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        self.angular = 0.0

        # Publisher
        self.position_target_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Start the keyboard listener
        self.get_logger().info("Keyboard control started. Use WASD for XY movement, SPACE/SHIFT for Z movement.")
        self.run_keyboard_listener()

    def run_keyboard_listener(self):
        keyboard.on_press(self.on_key_press)
        keyboard.on_release(self.on_key_release)

    def on_key_press(self, event):
        key = event.name
        if key == 'w':
            self.velocity[0] = 1.0  # Forward
        elif key == 'x':
            self.velocity[0] = -1.0  # Backward
        elif key == 'a':
            self.velocity[1] = 1.0  # Left
        elif key == 'd':
            self.velocity[1] = -1.0  # Right
        elif key == 'space':
            self.velocity[2] = 1.0  # Ascend
        elif key == 'shift':
            self.velocity[2] = -1.0  # Descend
        elif key == 'q':
            self.angular = 1.0
        elif key == 'e':
            self.angular = -1.0
        elif key == 's':
            self.velocity[0] = 0.0 
            self.velocity[1] = 0.0 
            self.velocity[2] = 0.0 
            self.angular = 0.0

    def on_key_release(self, event):
        key = event.name
        if key in ['w', 's']:
            self.velocity[0] = 0.0
        elif key in ['a', 'd']:
            self.velocity[1] = 0.0
        elif key in ['space', 'shift']:
            self.velocity[2] = 0.0

    def publish_velocity(self):
        position_target = PositionTarget()
        position_target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED
        position_target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ
        )

        position_target.velocity.x = self.velocity[0]
        position_target.velocity.y = self.velocity[1]
        position_target.velocity.z = self.velocity[2]
        position_target.yaw = self.angular

        self.position_target_pub.publish(position_target)
        self.get_logger().info(f"Velocity: {self.velocity}")

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()

    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()
        keyboard.unhook_all()  # Stop keyboard listener

if __name__ == '__main__':
    main()
