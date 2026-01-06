#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class OffboardKeepalive(Node):
    """
    Publishes OffboardControlMode + TrajectorySetpoint at a steady rate so Offboard is available.
    Does NOT arm, does NOT switch modes, does NOT take off.
    It holds the vehicle at the first valid local position it receives.
    """

    def __init__(self):
        super().__init__("offboard_keepalive")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.offboard_entered_ns = None
        self.exit_delay_s = 3.0
        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)

        self.have_home = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._status_cb, qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._local_pos_cb, qos)

        # 20 Hz keepalive
        self.create_timer(0.05, self._tick)

        self.get_logger().info("offboard_keepalive running, waiting for local position to latch hold setpoint")

    def _status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        # Always track current position until Offboard is entered
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.home_x = float(msg.x)
            self.home_y = float(msg.y)
            self.home_z = float(msg.z)
            if not self.have_home:
                self.have_home = True
                self.get_logger().info(
                    f"Latched hold setpoint NED: x={self.home_x:.2f}, y={self.home_y:.2f}, z={self.home_z:.2f}"
                )


    def _tick(self):
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.offboard_entered_ns is None:
                self.offboard_entered_ns = Clock().now().nanoseconds
                self.get_logger().info("Offboard detected, holding setpoint briefly before exit")
            else:
                elapsed_s = (Clock().now().nanoseconds - self.offboard_entered_ns) * 1e-9
                if elapsed_s >= self.exit_delay_s:
                    self.get_logger().info("Exiting keepalive, mission will take over")
                    rclpy.shutdown()
                    return
        

        t_us = int(Clock().now().nanoseconds / 1000)

        off = OffboardControlMode()
        off.timestamp = t_us
        off.position = True
        off.velocity = False
        off.acceleration = False
        off.attitude = False
        off.body_rate = False
        off.thrust_and_torque = False
        off.direct_actuator = False
        self.offboard_pub.publish(off)

        


        if not self.have_home:
            return

        sp = TrajectorySetpoint()
        sp.timestamp = t_us
        sp.position[0] = self.home_x
        sp.position[1] = self.home_y
        sp.position[2] = self.home_z
        sp.velocity[0] = float("nan")
        sp.velocity[1] = float("nan")
        sp.velocity[2] = float("nan")
        sp.acceleration[0] = float("nan")
        sp.acceleration[1] = float("nan")
        sp.acceleration[2] = float("nan")
        sp.yaw = self.home_yaw
        sp.yawspeed = float("nan")
        self.traj_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardKeepalive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
