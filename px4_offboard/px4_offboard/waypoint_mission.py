#!/usr/bin/env python3
"""
Waypoint Mission Control Node for PX4

This node implements an autonomous mission:
1. Takeoff to specified altitude (100m)
2. Fly to a GPS waypoint
3. Return to home location
4. Land

Integrates with mission_control_terminal.py for user interface.

Based on ARK Electronics ROS2_PX4_Offboard_Example velocity_control.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleGlobalPosition,
)
from std_msgs.msg import Bool, String

import math
from enum import Enum, auto


class MissionState(Enum):
    """Mission state machine states"""
    WAITING_FOR_START = auto()
    INITIALIZING = auto()
    ARMING = auto()
    TAKEOFF = auto()
    TRAVEL_TO_WAYPOINT = auto()
    HOVER_AT_WAYPOINT = auto()
    RETURN_HOME = auto()
    LAND = auto()
    MISSION_COMPLETE = auto()


class WaypointMission(Node):
    """ROS2 Node for autonomous waypoint mission control"""

    def __init__(self):
        super().__init__('waypoint_mission')

        # ============================================================
        # MISSION PARAMETERS - CONFIGURE YOUR MISSION HERE
        # ============================================================
        
        # Home location (Zurich - PX4 SITL default spawn)
        self.home_lat = 47.397972   # 47°23'52.7"N
        self.home_lon = 8.546111    # 8°32'46.0"E 8.546111
        self.home_alt = 488.0       # meters above sea level 
        
        # Target waypoint - SET YOUR DESTINATION HERE
        # Example: ~500m north of home location
        self.declare_parameter('target_lat', 47.397682)
        self.declare_parameter('target_lon', 8.543242)
        self.declare_parameter('takeoff_altitude', 100.0)
        self.declare_parameter('cruise_altitude', 100.0)
        self.declare_parameter('hover_time', 5.0)
        
        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        
        # Mission altitudes (relative to takeoff point)
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.cruise_altitude = self.get_parameter('cruise_altitude').value
        
        # Speed parameters
        self.ascent_speed = 5.0      # m/s vertical speed during takeoff
        self.cruise_speed = 10.0     # m/s horizontal cruise speed
        self.descent_speed = 3.0     # m/s vertical speed during landing
        
        # Position tolerance (when to consider waypoint "reached")
        self.horizontal_tolerance = 5.0   # meters
        self.vertical_tolerance = 2.0     # meters
        
        # Hover time at waypoint before returning (seconds)
        self.hover_time = self.get_parameter('hover_time').value
        
        # ============================================================
        # STATE VARIABLES
        # ============================================================
        
        self.state = MissionState.WAITING_FOR_START
        self.offboard_setpoint_counter = 0
        self.hover_start_time = None
        self.mission_start_requested = False
        
        # Vehicle state from PX4
        self.vehicle_local_position = None
        self.vehicle_global_position = None
        self.vehicle_status = None
        
        # Computed mission points (in local NED frame)
        self.takeoff_position = [0.0, 0.0, -self.takeoff_altitude]  # NED: negative Z is up
        self.target_position_ned = None  # Will be computed when we get GPS fix
        self.home_position_ned = [0.0, 0.0, -self.cruise_altitude]  # Return to origin at cruise alt
        self.landing_position = [0.0, 0.0, 0.0]  # Ground level at home
        
        # ============================================================
        # QOS PROFILE FOR PX4 COMMUNICATION
        # ============================================================
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ============================================================
        # PUBLISHERS
        # ============================================================
        
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Status publisher for terminal interface
        self.status_publisher = self.create_publisher(
            String, '/mission/status', qos_profile)

        # ============================================================
        # SUBSCRIBERS
        # ============================================================
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback, qos_profile)
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # Mission start subscriber (from terminal interface)
        self.mission_start_subscriber = self.create_subscription(
            Bool, '/mission/start',
            self.mission_start_callback, qos_profile)

        # ============================================================
        # TIMER - Main control loop at 10Hz
        # ============================================================
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Log startup info
        self.get_logger().info('='*60)
        self.get_logger().info('Waypoint Mission Node Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Home: {self.home_lat:.6f}, {self.home_lon:.6f}')
        self.get_logger().info(f'Target: {self.target_lat:.6f}, {self.target_lon:.6f}')
        self.get_logger().info(f'Takeoff altitude: {self.takeoff_altitude}m')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for start command from terminal...')
        
        # Send initial status
        self.publish_status("READY", "Waiting for start command... Press SPACE in terminal")

    # ================================================================
    # SUBSCRIBER CALLBACKS
    # ================================================================

    def vehicle_local_position_callback(self, msg):
        """Callback for local position updates"""
        self.vehicle_local_position = msg

    def vehicle_global_position_callback(self, msg):
        """Callback for global (GPS) position updates"""
        self.vehicle_global_position = msg
        
        # Compute target position in NED frame once we have GPS
        if self.target_position_ned is None and msg.lat != 0.0:
            self.compute_target_ned()

    def vehicle_status_callback(self, msg):
        """Callback for vehicle status updates"""
        self.vehicle_status = msg

    def mission_start_callback(self, msg):
        """Callback for mission start command from terminal"""
        if msg.data and self.state == MissionState.WAITING_FOR_START:
            self.get_logger().info('='*60)
            self.get_logger().info('MISSION START COMMAND RECEIVED!')
            self.get_logger().info('='*60)
            self.mission_start_requested = True
            self.state = MissionState.INITIALIZING
            self.publish_status("INITIALIZING", "Mission starting... Sending initial setpoints")

    # ================================================================
    # STATUS PUBLISHING
    # ================================================================

    def publish_status(self, state, info):
        """Publish status to terminal interface"""
        msg = String()
        msg.data = f"{state}|{info}"
        self.status_publisher.publish(msg)

    # ================================================================
    # COORDINATE CONVERSION
    # ================================================================

    def gps_to_ned(self, lat, lon, ref_lat, ref_lon):
        """
        Convert GPS coordinates to local NED frame relative to reference point.
        """
        # Earth radius in meters
        R = 6371000.0
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)
        
        # Calculate NED coordinates
        north = (lat_rad - ref_lat_rad) * R
        east = (lon_rad - ref_lon_rad) * R * math.cos(ref_lat_rad)
        
        return north, east

    def compute_target_ned(self):
        """Compute target waypoint in local NED coordinates"""
        if self.vehicle_global_position is None:
            return
            
        ref_lat = self.vehicle_global_position.lat
        ref_lon = self.vehicle_global_position.lon
        
        north, east = self.gps_to_ned(
            self.target_lat, self.target_lon,
            ref_lat, ref_lon
        )
        
        self.target_position_ned = [north, east, -self.cruise_altitude]
        
        distance = math.sqrt(north**2 + east**2)
        self.get_logger().info(f'Target computed: N={north:.1f}m, E={east:.1f}m, Dist={distance:.1f}m')

    # ================================================================
    # COMMAND PUBLISHERS
    # ================================================================

    def publish_offboard_control_mode(self, position=True, velocity=False):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        """Publish a position setpoint in NED frame"""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command to PX4"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """Send arm command"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0
        )
        self.get_logger().info('>>> ARM command sent')

    def disarm(self):
        """Send disarm command"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        self.get_logger().info('>>> DISARM command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.get_logger().info('>>> OFFBOARD MODE command sent')

    def land(self):
        """Command vehicle to land"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('>>> LAND command sent')

    # ================================================================
    # HELPER FUNCTIONS
    # ================================================================

    def get_distance_to_target(self, target):
        """Calculate 3D distance to target position"""
        if self.vehicle_local_position is None:
            return float('inf')
            
        dx = target[0] - self.vehicle_local_position.x
        dy = target[1] - self.vehicle_local_position.y
        dz = target[2] - self.vehicle_local_position.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def get_horizontal_distance(self, target):
        """Calculate horizontal distance to target"""
        if self.vehicle_local_position is None:
            return float('inf')
            
        dx = target[0] - self.vehicle_local_position.x
        dy = target[1] - self.vehicle_local_position.y
        
        return math.sqrt(dx**2 + dy**2)

    def get_altitude_agl(self):
        """Get current altitude above ground level"""
        if self.vehicle_local_position is None:
            return 0.0
        return -self.vehicle_local_position.z

    def is_at_position(self, target, h_tol=None, v_tol=None):
        """Check if vehicle is at target position within tolerance"""
        if self.vehicle_local_position is None:
            return False
            
        if h_tol is None:
            h_tol = self.horizontal_tolerance
        if v_tol is None:
            v_tol = self.vertical_tolerance
            
        h_dist = self.get_horizontal_distance(target)
        v_dist = abs(target[2] - self.vehicle_local_position.z)
        
        return h_dist < h_tol and v_dist < v_tol

    def is_armed(self):
        """Check if vehicle is armed"""
        if self.vehicle_status is None:
            return False
        return self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_in_offboard_mode(self):
        """Check if vehicle is in offboard mode"""
        if self.vehicle_status is None:
            return False
        return self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def calculate_yaw_to_target(self, target):
        """Calculate yaw angle to face target"""
        if self.vehicle_local_position is None:
            return 0.0
            
        dx = target[0] - self.vehicle_local_position.x
        dy = target[1] - self.vehicle_local_position.y
        
        return math.atan2(dy, dx)

    # ================================================================
    # MAIN STATE MACHINE
    # ================================================================

    def timer_callback(self):
        """Main control loop - runs at 10Hz"""
        
        # State machine
        if self.state == MissionState.WAITING_FOR_START:
            # Just wait, don't send anything to PX4 yet
            return
            
        # Always publish offboard control mode once mission starts
        self.publish_offboard_control_mode(position=True)
        
        # Wait for data from PX4
        if self.vehicle_local_position is None or self.vehicle_status is None:
            self.publish_status("CONNECTING", "Waiting for vehicle data...")
            return
            
        if self.state == MissionState.INITIALIZING:
            self.handle_initializing_state()
            
        elif self.state == MissionState.ARMING:
            self.handle_arming_state()
            
        elif self.state == MissionState.TAKEOFF:
            self.handle_takeoff_state()
            
        elif self.state == MissionState.TRAVEL_TO_WAYPOINT:
            self.handle_travel_state()
            
        elif self.state == MissionState.HOVER_AT_WAYPOINT:
            self.handle_hover_state()
            
        elif self.state == MissionState.RETURN_HOME:
            self.handle_return_home_state()
            
        elif self.state == MissionState.LAND:
            self.handle_land_state()
            
        elif self.state == MissionState.MISSION_COMPLETE:
            pass

    def handle_initializing_state(self):
        """INITIALIZING state - send initial setpoints"""
        self.publish_position_setpoint(0.0, 0.0, -self.takeoff_altitude)
        self.offboard_setpoint_counter += 1
        
        self.publish_status("INITIALIZING", 
            f"Sending initial setpoints... ({self.offboard_setpoint_counter}/10)")
        
        if self.offboard_setpoint_counter >= 10:
            self.get_logger().info('Initial setpoints sent, engaging offboard mode...')
            self.engage_offboard_mode()
            self.state = MissionState.ARMING
            self.offboard_setpoint_counter = 0
            self.publish_status("ARMING", "Engaging offboard mode...")

    def handle_arming_state(self):
        """ARMING state - arm the vehicle"""
        self.publish_position_setpoint(0.0, 0.0, -self.takeoff_altitude)
        self.offboard_setpoint_counter += 1
        
        if self.is_in_offboard_mode() and not self.is_armed():
            if self.offboard_setpoint_counter >= 10:
                self.get_logger().info('Offboard mode active, arming...')
                self.arm()
                self.offboard_setpoint_counter = 0
                self.publish_status("ARMING", "Sending arm command...")
                
        elif self.is_armed():
            self.get_logger().info('='*60)
            self.get_logger().info('ARMED - Starting takeoff!')
            self.get_logger().info('='*60)
            self.state = MissionState.TAKEOFF
            self.publish_status("TAKEOFF", "Armed! Ascending to 100m...")

    def handle_takeoff_state(self):
        """TAKEOFF state - ascend to cruise altitude"""
        self.publish_position_setpoint(
            self.takeoff_position[0],
            self.takeoff_position[1], 
            self.takeoff_position[2]
        )
        
        current_alt = self.get_altitude_agl()
        progress = min(100, int((current_alt / self.takeoff_altitude) * 100))
        
        self.publish_status("TAKEOFF", 
            f"Altitude: {current_alt:.1f}m / {self.takeoff_altitude}m ({progress}%)")
        
        if self.is_at_position(self.takeoff_position, h_tol=10.0, v_tol=3.0):
            if self.target_position_ned is not None:
                distance = self.get_horizontal_distance(self.target_position_ned)
                self.get_logger().info('='*60)
                self.get_logger().info(f'Takeoff complete! Heading to waypoint ({distance:.0f}m away)')
                self.get_logger().info('='*60)
                self.state = MissionState.TRAVEL_TO_WAYPOINT
                self.publish_status("TRAVELING", f"En route to waypoint... Distance: {distance:.0f}m")
            else:
                self.publish_status("TAKEOFF", "Waiting for GPS fix to compute target...")

    def handle_travel_state(self):
        """TRAVEL state - fly to waypoint"""
        if self.target_position_ned is None:
            return
            
        yaw = self.calculate_yaw_to_target(self.target_position_ned)
        
        self.publish_position_setpoint(
            self.target_position_ned[0],
            self.target_position_ned[1],
            self.target_position_ned[2],
            yaw
        )
        
        distance = self.get_horizontal_distance(self.target_position_ned)
        self.publish_status("TRAVELING", f"Distance to waypoint: {distance:.1f}m")
        
        if self.is_at_position(self.target_position_ned):
            self.get_logger().info('='*60)
            self.get_logger().info('Waypoint reached! Hovering...')
            self.get_logger().info('='*60)
            self.hover_start_time = self.get_clock().now()
            self.state = MissionState.HOVER_AT_WAYPOINT
            self.publish_status("HOVERING", f"At waypoint! Holding for {self.hover_time}s...")

    def handle_hover_state(self):
        """HOVER state - hold position at waypoint"""
        self.publish_position_setpoint(
            self.target_position_ned[0],
            self.target_position_ned[1],
            self.target_position_ned[2]
        )
        
        elapsed = (self.get_clock().now() - self.hover_start_time).nanoseconds / 1e9
        remaining = max(0, self.hover_time - elapsed)
        
        self.publish_status("HOVERING", f"Holding position... {remaining:.1f}s remaining")
        
        if elapsed >= self.hover_time:
            self.get_logger().info('='*60)
            self.get_logger().info('Hover complete! Returning home...')
            self.get_logger().info('='*60)
            self.state = MissionState.RETURN_HOME
            self.publish_status("RETURNING", "Heading back to home position...")

    def handle_return_home_state(self):
        """RETURN HOME state - fly back to home position"""
        yaw = self.calculate_yaw_to_target(self.home_position_ned)
        
        self.publish_position_setpoint(
            self.home_position_ned[0],
            self.home_position_ned[1],
            self.home_position_ned[2],
            yaw
        )
        
        distance = self.get_horizontal_distance(self.home_position_ned)
        self.publish_status("RETURNING", f"Distance to home: {distance:.1f}m")
        
        if self.is_at_position(self.home_position_ned):
            self.get_logger().info('='*60)
            self.get_logger().info('Home reached! Landing...')
            self.get_logger().info('='*60)
            self.state = MissionState.LAND
            self.publish_status("LANDING", "Descending...")

    def handle_land_state(self):
        """LAND state - descend and land"""
        self.publish_position_setpoint(
            self.landing_position[0],
            self.landing_position[1],
            self.landing_position[2]
        )
        
        current_alt = self.get_altitude_agl()
        self.publish_status("LANDING", f"Altitude: {current_alt:.1f}m")
        
        if current_alt < 1.0:
            self.land()
            
        if not self.is_armed():
            self.get_logger().info('='*60)
            self.get_logger().info('MISSION COMPLETE!')
            self.get_logger().info('='*60)
            self.state = MissionState.MISSION_COMPLETE
            self.publish_status("COMPLETE", "Mission finished successfully!")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission aborted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()