#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition


def wrap_pi(angle_rad: float) -> float:
    return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))


class MissionQGC(Node):
    """Simple offboard mission intended to cooperate with QGroundControl.

    Default behavior:
      - Continuously stream offboard setpoints.
      - Wait until the vehicle is Armed AND in Offboard mode (selected in QGC).
      - Then run: takeoff to altitude -> go to target lat/lon -> return home -> land.

    Optional behavior:VOLATILE
      - auto_arm: arm from this node (not recommended if QGC is your hub)
      - auto_offboard: switch to offboard from this node (not recommended if QGC is your hub)
    """

    def __init__(self):
        super().__init__('mission_qgc')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Parameters
        self.declare_parameter('target_lat_deg', 47.40025000)
        self.declare_parameter('target_lon_deg', 8.5069444)
        self.declare_parameter('altitude_m', 20.0)
        self.declare_parameter('acceptance_radius_m', 2.0)
        self.declare_parameter('acceptance_alt_m', 0.8)
        self.declare_parameter('cruise_yaw_deg', float('nan'))
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('auto_offboard', False)
        self.declare_parameter('auto_land', True)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._status_cb,
            qos,
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_cb,
            qos,
        )
        self.global_pos_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self._global_pos_cb,
            qos,
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos,
        )
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos,
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )

        # State
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.preflight_ok = False
        self.failsafe = False

        self.have_local = False
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0  # NED: positive down

        self.have_global = False
        self.lat_deg = 0.0
        self.lon_deg = 0.0

        self.home_latched = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_lat_deg = 0.0
        self.home_lon_deg = 0.0

        self.target_n = 0.0
        self.target_e = 0.0

        self.state = 'WAIT'
        self.setpoint_counter = 0

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 5.0)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info('mission_qgc node started')

    # Callbacks
    def _status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.preflight_ok = bool(msg.pre_flight_checks_pass)
        self.failsafe = bool(msg.failsafe)

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_x = float(msg.x)
        self.local_y = float(msg.y)
        self.local_z = float(msg.z)
        self.have_local = True

    def _global_pos_cb(self, msg: VehicleGlobalPosition):
        # PX4 uses degrees * 1e7
        self.lat_deg = float(msg.lat) * 1e-7
        self.lon_deg = float(msg.lon) * 1e-7
        self.have_global = True

    # Helpers
    def _armed(self) -> bool:
        return self.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _offboard(self) -> bool:
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0,
                                 param4: float = 0.0, param5: float = 0.0, param6: float = 0.0, param7: float = 0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _latch_home(self):
        self.home_x = float(self.local_x)
        self.home_y = float(self.local_y)
        self.home_z = float(self.local_z)
        self.home_lat_deg = float(self.lat_deg)
        self.home_lon_deg = float(self.lon_deg)
        self.home_latched = True

        # Convert target global to local N/E offsets (flat Earth approx)
        R = 6378137.0
        lat0 = math.radians(self.home_lat_deg)
        dlat = math.radians(float(self.get_parameter('target_lat_deg').value) - self.home_lat_deg)
        dlon = math.radians(float(self.get_parameter('target_lon_deg').value) - self.home_lon_deg)
        self.target_n = R * dlat
        self.target_e = R * math.cos(lat0) * dlon

        self.get_logger().info(
            f"Latched home: lat={self.home_lat_deg:.7f}, lon={self.home_lon_deg:.7f}, local(NED)=({self.home_x:.2f}, {self.home_y:.2f}, {self.home_z:.2f})"
        )
        self.get_logger().info(
            f"Target local offsets: N={self.target_n:.2f} m, E={self.target_e:.2f} m"
        )

    def _publish_offboard_stream(self, x_n: float, y_e: float, z_d: float, yaw_rad: float):
        now_us = int(Clock().now().nanoseconds / 1000)

        off = OffboardControlMode()
        off.timestamp = now_us
        off.position = True
        off.velocity = False
        off.acceleration = False
        self.offboard_mode_pub.publish(off)

        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.position[0] = float(x_n)
        sp.position[1] = float(y_e)
        sp.position[2] = float(z_d)

        sp.velocity[0] = float('nan')
        sp.velocity[1] = float('nan')
        sp.velocity[2] = float('nan')

        sp.acceleration[0] = float('nan')
        sp.acceleration[1] = float('nan')
        sp.acceleration[2] = float('nan')

        sp.yaw = float(yaw_rad)
        sp.yawspeed = float('nan')
        self.traj_pub.publish(sp)

    def _distance_xy(self, x_n: float, y_e: float) -> float:
        dx = float(self.local_x - x_n)
        dy = float(self.local_y - y_e)
        return float(math.sqrt(dx * dx + dy * dy))

    def _alt_error(self, z_d: float) -> float:
        return float(abs(self.local_z - z_d))

    # Main loop
    def _tick(self):
        if not (self.have_local and self.have_global):
            return

        if self.failsafe:
            self.get_logger().warn('Failsafe active, holding current setpoint stream')

        # Decide a yaw to use
        yaw_deg = float(self.get_parameter('cruise_yaw_deg').value)
        if math.isfinite(yaw_deg):
            yaw = wrap_pi(math.radians(yaw_deg))
        else:
            yaw = 0.0

        # Latch home once we are armed (or if auto_arm is used, soon after)
        if (not self.home_latched) and self._armed():
            self._latch_home()

        # Compute mission setpoints (in local NED)
        alt_m = float(self.get_parameter('altitude_m').value)
        target_z = self.home_z - abs(alt_m) if self.home_latched else -abs(alt_m)

        # Default hold position before mission starts
        hold_x = self.home_x if self.home_latched else self.local_x
        hold_y = self.home_y if self.home_latched else self.local_y
        hold_z = target_z if self.state in ['TAKEOFF', 'GOTO', 'RETURN'] else (self.home_z if self.home_latched else self.local_z)

        # Only publish once Offboard is active (or if auto_offboard is explicitly enabled)
        auto_offboard = bool(self.get_parameter('auto_offboard').value)
        if self._offboard() or auto_offboard:
                self._publish_offboard_stream(hold_x, hold_y, hold_z, yaw)
                self.setpoint_counter += 1

        auto_arm = bool(self.get_parameter('auto_arm').value)
        auto_land = bool(self.get_parameter('auto_land').value)

        # Optional: let node arm and switch modes, but only after some setpoints
        if auto_arm and (not self._armed()) and self.preflight_ok:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if auto_offboard and self._armed() and (not self._offboard()) and self.setpoint_counter > 20:
            # MAV_CMD_DO_SET_MODE: param1=1 (custom), param2=6 (PX4 offboard)
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # Mission gating: by default we only run when user has armed and selected Offboard in QGC
        if (not self._armed()) or (not self._offboard() and not auto_offboard):
            return

        if not self.home_latched:
            # Wait until home is latched (armed triggers latch)
            return

        r_xy = float(self.get_parameter('acceptance_radius_m').value)
        r_z = float(self.get_parameter('acceptance_alt_m').value)

        # State machine
        if self.state == 'WAIT':
            self.get_logger().info('Mission started')
            self.state = 'TAKEOFF'

        if self.state == 'TAKEOFF':
            self._publish_offboard_stream(self.home_x, self.home_y, target_z, yaw)
            if self._distance_xy(self.home_x, self.home_y) < r_xy and self._alt_error(target_z) < r_z:
                self.get_logger().info('Reached takeoff altitude')
                self.state = 'GOTO'

        elif self.state == 'GOTO':
            x = self.home_x + self.target_n
            y = self.home_y + self.target_e
            self._publish_offboard_stream(x, y, target_z, yaw)
            if self._distance_xy(x, y) < r_xy:
                self.get_logger().info('Reached target coordinate')
                self.state = 'RETURN'

        elif self.state == 'RETURN':
            self._publish_offboard_stream(self.home_x, self.home_y, target_z, yaw)
            if self._distance_xy(self.home_x, self.home_y) < r_xy:
                self.get_logger().info('Returned home')
                self.state = 'LAND'

        elif self.state == 'LAND':
            if auto_land:
                self.get_logger().info('Sending land command')
                self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'DONE'
            else:
                # If you prefer to land from QGC, set auto_land:=false
                self.get_logger().info('auto_land=false, hand over landing to QGC')
                self.state = 'DONE'

        elif self.state == 'DONE':
            # Keep streaming a safe hold setpoint so Offboard does not timeout
            self._publish_offboard_stream(self.home_x, self.home_y, target_z, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = MissionQGC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()