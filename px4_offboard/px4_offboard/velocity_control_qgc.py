#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
)


def wrap_pi(angle: float) -> float:
    return float(math.atan2(math.sin(angle), math.cos(angle)))


class QGCHubOffboardVelocity(Node):
    """
    QGC is the hub:
      - QGC handles arming, takeoff, land, and flight modes
      - This node only publishes OffboardControlMode and TrajectorySetpoint
      - It integrates /offboard_velocity_cmd into a position target ONLY in Offboard
    """

    def __init__(self):
        super().__init__('velocity_control_qgc')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos
        )
        self.offboard_velocity_sub = self.create_subscription(
            Twist, '/offboard_velocity_cmd', self.offboard_velocity_callback, qos
        )

        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.flight_check = False
        self.failsafe = False

        self.have_local_pos = False
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0

        self.true_yaw = 0.0

        self.cmd_vx_flu = 0.0
        self.cmd_vy_flu = 0.0
        self.cmd_vz_flu = 0.0
        self.cmd_yaw_rate = 0.0

        self.hold_latched = False
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0
        self.hold_yaw = 0.0

        self._last_t = None
        self._was_offboard = False

        publish_rate_hz = 50.0
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        self.get_logger().info('velocity_control_qgc started. Use QGC to arm and change modes.')

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

        is_offboard = (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        if is_offboard and not self._was_offboard:
            if self.have_local_pos:
                self.hold_x = float(self.local_x)
                self.hold_y = float(self.local_y)
                self.hold_z = float(self.local_z)
                self.hold_yaw = float(self.true_yaw)
                self.hold_latched = True
                self._last_t = None
                self.get_logger().info('Entered Offboard: latched current position for smooth handover.')
        self._was_offboard = is_offboard

    def local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_x = float(msg.x)
        self.local_y = float(msg.y)
        self.local_z = float(msg.z)
        self.have_local_pos = True

        if not self.hold_latched:
            self.hold_x = self.local_x
            self.hold_y = self.local_y
            self.hold_z = self.local_z
            self.hold_yaw = float(self.true_yaw)
            self.hold_latched = True

    def offboard_velocity_callback(self, msg: Twist) -> None:
        # Convert NED-style commands into body FLU
        self.cmd_vx_flu = -float(msg.linear.y)
        self.cmd_vy_flu = float(msg.linear.x)
        self.cmd_vz_flu = -float(msg.linear.z)
        self.cmd_yaw_rate = float(msg.angular.z)

    def attitude_callback(self, msg: VehicleAttitude) -> None:
        q = msg.q
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.true_yaw = -float(yaw)

    def timer_callback(self) -> None:
        if not self.hold_latched:
            return

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if not self.should_integrate():
            self._last_t = None
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is None:
            self._last_t = now
            return

        dt = float(np.clip(now - self._last_t, 0.0, 0.1))
        self._last_t = now

        cos_yaw = math.cos(self.true_yaw)
        sin_yaw = math.sin(self.true_yaw)

        v_world_x = self.cmd_vx_flu * cos_yaw - self.cmd_vy_flu * sin_yaw
        v_world_y = self.cmd_vx_flu * sin_yaw + self.cmd_vy_flu * cos_yaw
        v_world_z = self.cmd_vz_flu

        self.hold_x += float(v_world_x) * dt
        self.hold_y += float(v_world_y) * dt
        self.hold_z += float(v_world_z) * dt
        self.hold_yaw = wrap_pi(self.hold_yaw + float(self.cmd_yaw_rate) * dt)

    def should_integrate(self) -> bool:
        in_offboard = (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        armed = (self.arm_state == VehicleStatus.ARMING_STATE_ARMED)
        healthy = bool(self.flight_check) and not bool(self.failsafe)
        return in_offboard and armed and healthy

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_mode.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)

        msg.position[0] = float(self.hold_x)
        msg.position[1] = float(self.hold_y)
        msg.position[2] = float(self.hold_z)

        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')

        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')

        msg.yaw = float(self.hold_yaw)
        msg.yawspeed = float(self.cmd_yaw_rate)

        self.pub_traj.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QGCHubOffboardVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
