import os
import time
import argparse

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State, PositionTarget, Altitude
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

#if 'helpers' in os.getcwd():
#    from utils import is_px4, is_ardupilot
#else:
#    from helpers.utils import is_px4, is_ardupilot

from utils import is_px4, is_ardupilot


class ToL(Node):
    """Take off/Land/RTL"""

    def __init__(
        self,
        target_altitude: float = 1.5,
        takeoff_vel: float = 0.5,
        landing_vel: float = 0.5,
        init_mode: str = 'OFFBOARD',
        use_seconds: bool = False,
        target_seconds: float = 5.5,
    ):
        """Takeoff and Land node.

        Args:
            target_altitude (float, optional): takeoff distance. Defaults to 0.5.
            takeoff_vel (float, optional): takeoff velocity. Defaults to 0.5.
            landing_vel (float, optional): landing velocity (value * -1). Defaults to 0.5.
            init_mode (str, optional): working mode. Defaults to 'OFFBOARD'.
            use_seconds (bool, optional): use time for height estimation. Defaults to False.
        """
        super().__init__('takeoff_land')

        qos_profile = QoSProfile(depth=11)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_cb, qos_profile=qos_profile
        )
        # self.local_position_sub = self.create_subscription(
        #     PoseStamped,
        #     'mavros/local_position/pose',
        #     self.position_cb,
        #     qos_profile=qos_profile,
        # )
        if is_px4():
            self.altitude_sub = self.create_subscription(
                Altitude,
                'mavros/altitude',
                self.altitude_cb,
                qos_profile=qos_profile,
            )
        elif is_ardupilot():
            self.altitude_sub = self.create_subscription(
                Float64,
                'mavros/global_position/rel_alt',
                self.altitude_cb,
                qos_profile=qos_profile,
            )

        # Publishers
        self.local_target_pub = self.create_publisher(
            PositionTarget, 'mavros/setpoint_raw/local', 10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Variables
        self.current_state = State()
        self.target_altitude = target_altitude
        self.target_seconds = target_seconds
        self.takeoff_vel = takeoff_vel
        self.landing_vel = landing_vel * -1
        self.init_mode = init_mode
        self.use_seconds = use_seconds

        self.current_altitude = 0.0
        self.reached_altitude = False
        self.decrease_speed = False
        self.start_landing = False
        self.is_takeoff = False
        braking_coeff = 3 if self.takeoff_vel > 5 else 4
        self.braking_altitude = abs(
            self.target_altitude - (self.takeoff_vel / braking_coeff + 0.5)
        )
        self.braking_vel = self.takeoff_vel / 2

        #print(f"seconds for takeoff { self.target_seconds }")
        self.takeoff_time_estimate = time.time() + self.target_seconds #time.time() + 6.0 #(
        #    time.time() + (self.target_altitude / self.takeoff_vel) + 5.5
        #)
        # self.landing_time_estimate = self.target_altitude / self.landing_vel
        self.takeoff_timer = 0.0
        self.landing_timer = 0.0
        print(self.takeoff_time_estimate)

        self.target = PositionTarget()
        self.target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )

    def state_cb(self, msg: State):
        """Current state"""
        self.current_state = msg

    def position_cb(self, msg: PoseStamped):
        """Checks if the drone has reached take off or landing height"""
        # self.current_altitude = msg.pose.position.z

        # # Check if takeoff altitude has been reached
        # if self.is_takeoff and (
        #     self.target_altitude <= self.current_altitude
        #     or abs(self.target_altitude - self.current_altitude) <= 0.5
        # ):
        #     self.reached_altitude = True
        #     self.get_logger().info(
        #         f"Take off is done. Height: {self.current_altitude:.2f}"
        #     )

        # # if approaching the target altitude decrease speed
        # if (
        #     is_ardupilot()
        #     and self.is_takeoff
        #     and not self.decrease_speed
        #     and self.current_altitude >= self.braking_altitude
        # ):
        #     self.get_logger().info(
        #         f"Braking. Height: {self.current_altitude:.2f}; Vel: {self.braking_vel}"
        #     )
        #     self.decrease_speed = True

        # # Check if start landing (turn landing mode on the height 2 m)
        # if (
        #     not self.is_takeoff
        #     and not self.start_landing
        #     and self.current_altitude - 2.0 <= 0.5
        # ):
        #     self.start_landing = True
        #     self.get_logger().info(
        #         f"Started landing. Height: {self.current_altitude:.2f}; Vel: {self.landing_vel}"
        #     )
        pass

    def altitude_cb(self, msg: Altitude):
        """Checks if the drone has reached take off or landing height"""
        self.current_altitude = (
            msg.relative if is_px4() else msg.data
        )  # relative to home

        # Check if takeoff altitude has been reached
        if not self.use_seconds:
            if self.is_takeoff and (
                self.target_altitude <= self.current_altitude
                or abs(self.target_altitude - self.current_altitude) <= 0.5
            ):
                self.reached_altitude = True
                self.get_logger().info(
                    f"Take off is done. Height: {self.current_altitude:.2f}"
                )
            # if approaching the target altitude decrease speed
            if (
                # is_ardupilot() and
                self.is_takeoff
                and not self.decrease_speed
                and self.current_altitude >= self.braking_altitude
            ):
                self.get_logger().info(
                    f"Braking. Height: {self.current_altitude:.2f}; Vel: {self.braking_vel}"
                )
                self.decrease_speed = True
        else:
            print(self.takeoff_timer)
            if self.is_takeoff and (self.takeoff_time_estimate <= self.takeoff_timer):
                self.reached_altitude = True
                self.get_logger().info(
                    f"Take off is done. Height: {self.current_altitude:.2f}"
                )
            # if (
            #     self.is_takeoff
            #     and not self.decrease_speed
            #     and self.takeoff_timer * self.takeoff_vel >= self.braking_altitude
            # ):
            #     self.get_logger().info(
            #         f"Braking. Height: {self.current_altitude:.2f}; Vel: {self.braking_vel}"
            #     )
            #     self.decrease_speed = True

        # Check if start landing (turn landing mode on the height 2 m)
        if not self.use_seconds:
            if (
                not self.is_takeoff
                and not self.start_landing
                and self.current_altitude - 2.0 <= 0.5
            ):
                self.start_landing = True
                self.get_logger().info(
                    f"Started landing. Height: {self.current_altitude:.2f}; Vel: {self.landing_vel}"
                )
        else:
            if (
                not self.is_takeoff
                and not self.start_landing
                and self.landing_vel * self.landing_timer * -1 - 0.5 <= 0.2
            ):
                self.start_landing = True
                self.get_logger().info(
                    f"Started landing. Height: {self.current_altitude:.2f}; Vel: {self.landing_vel}"
                )

    def arming(self, arm: bool = False):
        """Arm or disarm the drone

        Args:
            arm (bool, optional): `True` - arm, `False` - disarm.
            Defaults to False.
        """
        while not (arm and self.current_state.armed):
            if self.arming_client.wait_for_service(timeout_sec=1.0):
                arming_call = CommandBool.Request()
                arming_call.value = arm
                future = self.arming_client.call_async(arming_call)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().success:
                    self.get_logger().info('Success')
                    break
                self.get_logger().warn('Failed. Retrying...')
            time.sleep(0.5)

    def set_mode(self, mode: str = 'OFFBOARD'):
        """Set mode

        Args:
            mode (str, optional): mode to use. Defaults to 'OFFBOARD'.
        """
        while self.current_state.mode != mode:
            if self.set_mode_client.wait_for_service(timeout_sec=1.0):
                mode_call = SetMode.Request()
                mode_call.custom_mode = mode
                future = self.set_mode_client.call_async(mode_call)
                rclpy.spin_until_future_complete(self, future)
                if future.result() and future.result().mode_sent:
                    self.get_logger().info(f'Set {mode} successfully.')
                    break
                self.get_logger().warn(f'Setting {mode} failed. Retrying...')
            time.sleep(0.5)

    def prearm(self):
        """Set take off velocity and publish points for OFFBOARD mode"""
        # Take off velocity
        self.target.velocity.z = self.takeoff_vel

        self.get_logger().info("Prearm. Setting points...")
        # Send a few setpoints before starting OFFBOARD mode
        for _ in range(100):
            self.local_target_pub.publish(self.target)
            time.sleep(0.01)

    def takeoff(self):
        """Take off"""
        self.is_takeoff = True
        if is_px4():
            self.prearm()
        self.set_mode(self.init_mode)
        self.arming(True)

        if is_ardupilot():
            time.sleep(5)
            request = CommandTOL.Request()
            request.altitude = 0.5
            f = self.takeoff_client.call_async(request)
            rclpy.spin_until_future_complete(self, f)
            time.sleep(2)

        self.get_logger().info(
            f"Take off... Dist: {self.target_altitude}; Vel: {self.takeoff_vel}"
        )

        self.target.velocity.z = self.takeoff_vel
        while not self.reached_altitude:
            self.takeoff_timer = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.decrease_speed:
                self.target.velocity.z = self.braking_vel
                # self.local_target_pub.publish(self.target)
            if not self.reached_altitude:
                self.local_target_pub.publish(self.target)
                time.sleep(0.01)

        if is_px4():
            self.target.velocity.z = 0.1  # to keep same altitude
        elif is_ardupilot():
            self.target.velocity.z = 0.0  # to stop ascending
        self.local_target_pub.publish(self.target)

    def land(self):
        """Land the drone at the current XY coordinates"""
        self.is_takeoff = False
        self.get_logger().info(
            f"Descending... Dist: {self.current_altitude:.2f}; Vel: {self.landing_vel}"
        )

        self.target.velocity.z = self.landing_vel
        while not self.start_landing:
            self.landing_timer = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)
            if not self.start_landing:
                self.local_target_pub.publish(self.target)
                time.sleep(0.01)

        self.target.velocity.z = 0.1
        self.local_target_pub.publish(self.target)

        self.set_mode('AUTO.LAND' if is_px4() else 'LAND')
        time.sleep(6.5)
        self.arming(False)

    def rtl(self):
        """Initiate Return To Launch program"""
        self.is_takeoff = False
        self.get_logger().info('Return to launch...')
        self.set_mode('AUTO.RTL' if is_px4() else 'RTL')
        self.land()


def run_takeoff(
    option: str = None,
    target_altitude: float = 0.5,
    takeoff_vel: float = 0.5,
    landing_vel: float = 0.5,
    init_mode: str = 'OFFBOARD',
    use_seconds: bool = False,
    target_seconds: float = 5.5,
):
    """Run one basic functionality (Takeoff/Land/RTL), based on args

    Args:
        option (str, optional): func from terminal line. Defaults to None.
        target_altitude (float, optional): takeoff distance. Defaults to 0.5.
        takeoff_vel (float, optional): takeoff velocity. Defaults to 0.5.
        landing_vel (float, optional): landing velocity (value * -1). Defaults to 0.5.
        init_mode (str, optional): working mode. Defaults to 'OFFBOARD'.
        use_seconds (bool, optional): use time for height estimation. Defaults to False.
    """
    rclpy.init()
    taking_off = ToL(target_altitude, takeoff_vel, landing_vel, init_mode, use_seconds, target_seconds)
    start_timer = time.time()
    if option == '0' or option.lower() == 't':
        taking_off.takeoff()
    elif option == '1' or option.lower() == 'l':
        taking_off.land()
    elif option == '2' or option.lower() == 'rtl':
        taking_off.rtl()
    print(f"Total time: {(time.time() - start_timer):.2f} s")
    taking_off.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    DEFAULT_MODE = 'OFFBOARD' if is_px4() else 'GUIDED'

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-f',
        '--func',
        dest='func',
        help="what function to perform (takeoff: 0 or t, land: 1 or l, return to launch: 2 or rtl)",
        type=str,
        default=None,
    )
    parser.add_argument(
        '-a',
        '--altitude',
        dest='altitude',
        help="insert target takeoff altitude",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        '-t',
        '--takeoff_vel',
        dest='takeoff_vel',
        help="insert takeoff velocity",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        '-l',
        '--land_vel',
        dest='land_vel',
        help="insert landing velocity (positive value)",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        '-m',
        '--mode',
        dest='mode',
        help="insert main working mode (set to after takeoff)",
        type=str,
        default=DEFAULT_MODE,
    )
    parser.add_argument(
        '-s',
        '--use_seconds',
        dest='use_seconds',
        help="True if use time estimation, calculated based on a/t",
        type=bool,
        default=False,
    )
    parser.add_argument(
        '-ts',
        '--target_seconds',
        dest='target_seconds',
        help="True if use time estimation, calculated based time.",
        type=float,
        default=5.5,
    )

    args = parser.parse_args()

    run_takeoff(
        option=args.func,
        target_altitude=args.altitude,
        takeoff_vel=args.takeoff_vel,
        landing_vel=args.land_vel,
        init_mode=args.mode,
        use_seconds=args.use_seconds,
        target_seconds=args.target_seconds,
    )
