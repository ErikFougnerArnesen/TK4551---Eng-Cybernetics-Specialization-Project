#!/usr/bin/env python3
"""
Mission Control Terminal Interface

This node provides a terminal-based interface for the waypoint mission.
Based on control.py from the original ARK Electronics example.

Controls:
    SPACE  - Start Mission
    Q      - Quit

Based on ARK Electronics ROS2_PX4_Offboard_Example control.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String

import sys
import tty
import termios
import select


# ASCII Art Banner
BANNER = """
╔═══════════════════════════════════════════════════════════════════════════════╗
║                                                                               ║
║     ██████╗ ██╗  ██╗██╗  ██╗    ██╗    ██╗ █████╗ ██╗   ██╗██████╗  ██████╗   ║
║     ██╔══██╗╚██╗██╔╝██║  ██║    ██║    ██║██╔══██╗╚██╗ ██╔╝██╔══██╗██╔═══██╗  ║
║     ██████╔╝ ╚███╔╝ ███████║    ██║ █╗ ██║███████║ ╚████╔╝ ██████╔╝██║   ██║  ║
║     ██╔═══╝  ██╔██╗ ╚════██║    ██║███╗██║██╔══██║  ╚██╔╝  ██╔═══╝ ██║   ██║  ║
║     ██║     ██╔╝ ██╗     ██║    ╚███╔███╔╝██║  ██║   ██║   ██║     ╚██████╔╝  ║
║     ╚═╝     ╚═╝  ╚═╝     ╚═╝     ╚══╝╚══╝ ╚═╝  ╚═╝   ╚═╝   ╚═╝      ╚═════╝   ║
║                                                                               ║
║                    WAYPOINT MISSION - OFFBOARD CONTROL                        ║
║                                                                               ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                               ║
║   Mission Profile:                                                            ║
║   ┌─────────────────────────────────────────────────────────────────────┐     ║
║   │  1. TAKEOFF      →  Ascend to 100m altitude                         │     ║
║   │  2. TRAVEL       →  Fly to target waypoint                          │     ║
║   │  3. HOVER        →  Hold position for 5 seconds                     │     ║
║   │  4. RETURN HOME  →  Fly back to starting position                   │     ║
║   │  5. LAND         →  Descend and land safely                         │     ║
║   └─────────────────────────────────────────────────────────────────────┘     ║
║                                                                               ║
╠═══════════════════════════════════════════════════════════════════════════════╣
║                                                                               ║
║   CONTROLS:                                                                   ║
║   ─────────────────────────────────────────────────────────────────────────   ║
║                                                                               ║
║       [ SPACE ]  ─────────────────────────  START MISSION                     ║
║                                                                               ║
║       [   Q   ]  ─────────────────────────  QUIT                              ║
║                                                                               ║
╚═══════════════════════════════════════════════════════════════════════════════╝
"""

STATUS_TEMPLATE = """
┌─────────────────────────────────────────────────────────────────────────────┐
│  STATUS: {status:<64} │
│  {info:<72} │
└─────────────────────────────────────────────────────────────────────────────┘

   >>> Press SPACE to start mission | Press Q to quit <<<
"""


class MissionControlTerminal(Node):
    """Terminal interface for waypoint mission control"""

    def __init__(self):
        super().__init__('mission_control_terminal')

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher to send start command to mission node
        self.mission_start_publisher = self.create_publisher(
            Bool, '/mission/start', qos_profile)

        # Subscriber to receive status updates from mission node
        self.mission_status_subscriber = self.create_subscription(
            String, '/mission/status', self.status_callback, qos_profile)

        # State
        self.mission_started = False
        self.current_status = "WAITING"
        self.current_info = "Waiting for user to start mission..."

        # Terminal settings for raw input
        self.settings = termios.tcgetattr(sys.stdin)

        # Print banner
        self.clear_screen()
        print(BANNER)
        self.print_status()

        # Timer to check for keyboard input
        self.timer = self.create_timer(0.1, self.timer_callback)

    def clear_screen(self):
        """Clear terminal screen"""
        print("\033[2J\033[H", end="")

    def print_status(self):
        """Print current status"""
        print(STATUS_TEMPLATE.format(
            status=self.current_status,
            info=self.current_info
        ))

    def status_callback(self, msg):
        """Callback for mission status updates"""
        # Parse status message (format: "STATE|Info message")
        parts = msg.data.split('|', 1)
        if len(parts) == 2:
            self.current_status = parts[0]
            self.current_info = parts[1]
        else:
            self.current_status = msg.data
            self.current_info = ""

        # Refresh display
        self.clear_screen()
        print(BANNER)
        self.print_status()

    def get_key(self):
        """Get keyboard input (non-blocking)"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        """Check for keyboard input"""
        key = self.get_key()

        if key == ' ':  # Space bar
            if not self.mission_started:
                self.start_mission()
            else:
                self.get_logger().info('Mission already started!')

        elif key == 'q' or key == 'Q':
            self.quit()

    def start_mission(self):
        """Send start command to mission node"""
        msg = Bool()
        msg.data = True
        self.mission_start_publisher.publish(msg)
        self.mission_started = True

        self.current_status = "STARTING..."
        self.current_info = "Mission start command sent!"

        self.clear_screen()
        print(BANNER)
        self.print_status()

        self.get_logger().info('Mission start command sent!')

    def quit(self):
        """Clean exit"""
        self.clear_screen()
        print("\n" + "="*60)
        print("  Mission Control Terminal - Shutting down...")
        print("="*60 + "\n")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        raise SystemExit

    def destroy_node(self):
        """Clean up on shutdown"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionControlTerminal()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()