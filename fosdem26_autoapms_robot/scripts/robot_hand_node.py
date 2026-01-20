#!/usr/bin/env python3
"""
Robot Hand Visualization Node for FOSDEM 2026 Demo.

This node simulates a robot hand attached to an arm that moves in a circular arc.
The hand position is controlled via velocity commands and visualized in the terminal.
"""

import math
import sys
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from fosdem26_autoapms_interfaces.msg import RobotVelocity, RobotPosition


@dataclass
class TerminalConfig:
    """Terminal display configuration."""
    width: int = 80
    text_height: int = 6      # Height for FOSDEM text area
    robot_height: int = 15    # Height for robot hand area


class RobotHandVisualizer:
    """Handles the terminal visualization of the robot hand."""

    # Tesla Bot style hand - center position (vertical)
    # Sleek mechanical hand with articulated finger segments
    # Thumb always on the RIGHT side (right hand viewed from front)
    # All hand art is 28 chars wide, centered
    HAND_CENTER = [
        "  ┌┐ ┌┐ ┌┐ ┌┐              ",
        "  ││ ││ ││ ││              ",
        "  ├┤ ├┤ ├┤ ├┤              ",
        "  ││ ││ ││ ││              ",
        "  └┴─┴┴─┴┴─┴┘     ┌┐       ",
        "  ┌─────────────┐ ││       ",
        "  │ ░░░░░░░░░░░ │ ├┤       ",
        "  │ ░░░░░░░░░░░ │ ││       ",
        "  └─────────────┴─┴┘       ",
        "   │ ▓▓▓▓▓▓▓▓▓ │           ",
        "   │ ▓▓▓▓▓▓▓▓▓ │           ",
        "   └───────────┘           ",
    ]
    
    # Hand barely tilted left (between center and slight)
    HAND_LEFT_LESS = [
        "   ┌┐ ┌┐ ┌┐ ┌┐             ",
        "   ││ ││ ││ ││             ",
        "   ├┤ ├┤ ├┤ ├┤             ",
        "    ││ ││ ││ ││            ",
        "    └┴─┴┴─┴┴─┴┘    ┌┐      ",
        "    ┌─────────────┐ ││     ",
        "    │ ░░░░░░░░░░░ │ ├┤     ",
        "    │ ░░░░░░░░░░░ │ ││     ",
        "    └─────────────┴─┴┘     ",
        "     │ ▓▓▓▓▓▓▓▓▓ │         ",
        "      │ ▓▓▓▓▓▓▓▓▓ │        ",
        "      └───────────┘        ",
    ]

    # Hand slightly tilted left (entire hand leans left)
    HAND_LEFT_SLIGHT = [
        "   ┌┐ ┌┐ ┌┐ ┌┐             ",
        "    ││ ││ ││ ││            ",
        "    ├┤ ├┤ ├┤ ├┤            ",
        "     ││ ││ ││ ││           ",
        "     └┴─┴┴─┴┴─┴┘    ┌┐     ",
        "     ┌─────────────┐ ││    ",
        "      │ ░░░░░░░░░░░ │├┤    ",
        "      │ ░░░░░░░░░░░ │ ││   ",
        "      └─────────────┴─┴┘   ",
        "       │ ▓▓▓▓▓▓▓▓▓ │       ",
        "        │ ▓▓▓▓▓▓▓▓▓ │      ",
        "        └───────────┘      ",
    ]

    # Hand tilted left (entire hand leans more left)
    HAND_LEFT = [
        "   ┌┐ ┌┐ ┌┐ ┌┐             ",
        "    ││ ││ ││ ││            ",
        "     ├┤ ├┤ ├┤ ├┤           ",
        "      ││ ││ ││ ││          ",
        "       └┴─┴┴─┴┴─┴┘    ┌┐   ",
        "       ┌─────────────┐ ││  ",
        "       │ ░░░░░░░░░░░ │ ├┤  ",
        "        │ ░░░░░░░░░░░ │ ││ ",
        "        └─────────────┴─┴┘ ",
        "         │ ▓▓▓▓▓▓▓▓▓ │     ",
        "          │ ▓▓▓▓▓▓▓▓▓ │    ",
        "          └───────────┘    ",
    ]
    
    # Hand barely tilted right (between center and slight)
    HAND_RIGHT_LESS = [
        "  ┌┐ ┌┐ ┌┐ ┌┐             ",
        "  ││ ││ ││ ││             ",
        "  ├┤ ├┤ ├┤ ├┤             ",
        " ││ ││ ││ ││              ",
        " └┴─┴┴─┴┴─┴┘      ┌┐      ",
        " ┌─────────────┐ ││        ",
        " │ ░░░░░░░░░░░ │ ├┤        ",
        " │ ░░░░░░░░░░░ │ ││        ",
        " └─────────────┴─┴┘        ",
        "  │ ▓▓▓▓▓▓▓▓▓ │            ",
        "  │ ▓▓▓▓▓▓▓▓▓ │            ",
        "  └───────────┘            ",
    ]

    # Hand slightly tilted right (entire hand leans right)
    HAND_RIGHT_SLIGHT = [
        "     ┌┐ ┌┐ ┌┐ ┌┐           ",
        "    ││ ││ ││ ││            ",
        "    ├┤ ├┤ ├┤ ├┤            ",
        "   ││ ││ ││ ││             ",
        "   └┴─┴┴─┴┴─┴┘      ┌┐     ",
        "   ┌─────────────┐ ││      ",
        "  │ ░░░░░░░░░░░ │  ├┤      ",
        "  │ ░░░░░░░░░░░ │ ││       ",
        "  └─────────────┴─┘        ",
        "   │ ▓▓▓▓▓▓▓▓▓ │           ",
        "  │ ▓▓▓▓▓▓▓▓▓ │            ",
        "  └───────────┘            ",
    ]

    # Hand tilted right (entire hand leans more right)
    HAND_RIGHT = [
        "       ┌┐ ┌┐ ┌┐ ┌┐         ",
        "      ││ ││ ││ ││          ",
        "     ├┤ ├┤ ├┤ ├┤           ",
        "    ││ ││ ││ ││            ",
        "   └┴─┴┴─┴┴─┴┘      ┌┐     ",
        "  ┌─────────────┐  ││      ",
        "  │ ░░░░░░░░░░░ │  ├┤      ",
        " │ ░░░░░░░░░░░ │  ││       ",
        " └─────────────┴─┴┘        ",
        "  │ ▓▓▓▓▓▓▓▓▓ │            ",
        " │ ▓▓▓▓▓▓▓▓▓ │             ",
        " └───────────┘             ",
    ]

    # Arm segments for different angles (adjusted for rotation)
    ARM_CENTER = "   │ ▓▓▓▓▓▓▓▓▓ │           "
    ARM_LEFT_LESS = "      │ ▓▓▓▓▓▓▓▓▓ │        "
    ARM_LEFT_SLIGHT = "        │ ▓▓▓▓▓▓▓▓▓ │      "
    ARM_LEFT = "          │ ▓▓▓▓▓▓▓▓▓ │    "
    ARM_RIGHT_LESS = "  │ ▓▓▓▓▓▓▓▓▓ │            "
    ARM_RIGHT_SLIGHT = "  │ ▓▓▓▓▓▓▓▓▓ │            "
    ARM_RIGHT = " │ ▓▓▓▓▓▓▓▓▓ │             "

    def __init__(self, config: TerminalConfig):
        self.config = config
        self.frame_buffer = []
        self._shine_offset = 0

    def _get_hand_art(self, position: float) -> tuple[list[str], str]:
        """Get the appropriate hand art and arm segment based on position/angle."""
        if position < -0.6:
            return self.HAND_LEFT, self.ARM_LEFT
        elif position < -0.35:
            return self.HAND_LEFT_SLIGHT, self.ARM_LEFT_SLIGHT
        elif position < -0.1:
            return self.HAND_LEFT_LESS, self.ARM_LEFT_LESS
        elif position > 0.6:
            return self.HAND_RIGHT, self.ARM_RIGHT
        elif position > 0.35:
            return self.HAND_RIGHT_SLIGHT, self.ARM_RIGHT_SLIGHT
        elif position > 0.1:
            return self.HAND_RIGHT_LESS, self.ARM_RIGHT_LESS
        else:
            return self.HAND_CENTER, self.ARM_CENTER

    def _get_fosdem_text(self) -> list[str]:
        """Generate FOSDEM 26 text with shiny animation effect."""
        text_lines = [
            "███████╗ ██████╗ ███████╗██████╗ ███████╗███╗   ███╗    ██████╗  ██████╗ ",
            "██╔════╝██╔═══██╗██╔════╝██╔══██╗██╔════╝████╗ ████║    ╚════██╗██╔════╝ ",
            "█████╗  ██║   ██║███████╗██║  ██║█████╗  ██╔████╔██║     █████╔╝███████╗ ",
            "██╔══╝  ██║   ██║╚════██║██║  ██║██╔══╝  ██║╚██╔╝██║    ██╔═══╝ ██╔═══██╗",
            "██║     ╚██████╔╝███████║██████╔╝███████╗██║ ╚═╝ ██║    ███████╗╚██████╔╝",
            "╚═╝      ╚═════╝ ╚══════╝╚═════╝ ╚══════╝╚═╝     ╚═╝    ╚══════╝ ╚═════╝ ",
        ]
        return text_lines

    def _apply_shine_effect(self, text: str, line_index: int) -> str:
        """Apply a shiny wave effect to text."""
        result = []
        shine_pos = (self._shine_offset - line_index * 2) % (len(text) + 20)

        for i, char in enumerate(text):
            if char == ' ' or char == '\n':
                result.append(char)
            elif shine_pos - 3 <= i <= shine_pos + 3:
                # Bright white shine
                distance = abs(i - shine_pos)
                if distance == 0:
                    result.append(f"\033[97;1m{char}\033[0m")  # Bright white bold
                elif distance <= 1:
                    result.append(f"\033[96;1m{char}\033[0m")  # Bright cyan
                elif distance <= 2:
                    result.append(f"\033[36m{char}\033[0m")    # Cyan
                else:
                    result.append(f"\033[34m{char}\033[0m")    # Blue
            else:
                result.append(f"\033[35m{char}\033[0m")  # Magenta (base color)

        return ''.join(result)

    def _calculate_hand_position(self, normalized_pos: float) -> tuple[int, int]:
        """
        Calculate hand position based on circular arc motion.

        The arm swings from left to right in a circular arc.
        normalized_pos: -1.0 (left) to 1.0 (right)
        Returns (x, y) position in terminal coordinates.
        """
        # Maximum angle at full position (-1 or 1)
        max_angle_deg = 15
        max_angle_rad = math.radians(max_angle_deg)
        
        # Calculate arm length based on width: at max angle, hand should reach near edge
        # At 60 degrees: x_offset = arm_length * sin(60) ≈ 0.866 * arm_length
        # We want x_offset to be about (width/2 - margin)
        margin = 15  # Keep hand away from edges
        max_x_offset = (self.config.width // 2) - margin
        arc_radius = max_x_offset / math.sin(max_angle_rad)
        
        center_x = self.config.width // 2
        center_y = self.config.robot_height + arc_radius - 3

        # Convert normalized position to angle
        angle = normalized_pos * max_angle_rad

        # Calculate position on arc
        x = int(center_x + arc_radius * math.sin(angle))
        y = int(center_y - arc_radius * math.cos(angle))

        return x, y

    def render_frame(self, position: float, velocity: float) -> str:
        """Render a complete frame with FOSDEM text on top and robot hand below."""
        # Initialize frame buffer for robot area
        self.frame_buffer = [[' ' for _ in range(self.config.width)]
                             for _ in range(self.config.robot_height)]

        # Calculate hand position
        hand_x, hand_y = self._calculate_hand_position(position)

        # Draw the arm extending from below the screen
        self._draw_arm(hand_x, hand_y, position)

        # Draw the hand (with tilt based on position)
        self._draw_hand(hand_x, hand_y, position)

        # Get FOSDEM text
        fosdem_text = self._get_fosdem_text()

        # Compose final output using explicit cursor positioning to avoid scrollback pollution
        output = []
        row = 1

        # Top padding
        output.append(f"\033[{row};1H" + " " * self.config.width)
        row += 1

        # FOSDEM text (centered)
        for i, line in enumerate(fosdem_text):
            padding = (self.config.width - len(line)) // 2
            padded_line = ' ' * padding + line + ' ' * padding
            styled_line = self._apply_shine_effect(padded_line[:self.config.width], i)
            output.append(f"\033[{row};1H" + styled_line)
            row += 1

        # Bottom padding after text
        output.append(f"\033[{row};1H" + " " * self.config.width)
        row += 1

        # Robot hand visualization
        for y in range(self.config.robot_height):
            robot_line = ''.join(self.frame_buffer[y])
            output.append(f"\033[{row};1H" + robot_line.ljust(self.config.width))
            row += 1

        # Update shine animation
        self._shine_offset = (self._shine_offset + 2) % 100

        # Status bar
        output.append(f"\033[{row};1H" + "─" * self.config.width)
        row += 1
        status = f"\033[90m Position: {position:+.2f} | Velocity: {velocity:+.2f} │ Press Ctrl+C to exit\033[0m"
        output.append(f"\033[{row};1H" + status.ljust(self.config.width))

        return ''.join(output)

    def _draw_hand(self, x: int, y: int, position: float):
        """Draw the robot hand at the specified position with tilt."""
        hand_art, _ = self._get_hand_art(position)
        hand_width = len(hand_art[0])

        for i, line in enumerate(hand_art):
            draw_y = y - len(hand_art) + i + 1
            if 0 <= draw_y < self.config.robot_height:
                for j, char in enumerate(line):
                    draw_x = x - hand_width // 2 + j
                    if 0 <= draw_x < self.config.width and char != ' ':
                        self.frame_buffer[draw_y][draw_x] = char

    def _draw_arm(self, hand_x: int, hand_y: int, position: float):
        """Draw the arm extending from the hand to below the screen."""
        _, arm_segment = self._get_hand_art(position)
        arm_width = len(arm_segment)

        # Calculate arm angle based on position
        angle = position * math.radians(60)

        for y in range(hand_y, self.config.robot_height + 5):
            if y >= self.config.robot_height:
                break

            # Calculate x offset based on angle and distance from hand
            distance = y - hand_y
            x_offset = int(distance * math.tan(angle) * 0.3)
            draw_x = hand_x + x_offset

            # Draw arm segment
            for j, char in enumerate(arm_segment):
                final_x = draw_x - arm_width // 2 + j
                if 0 <= final_x < self.config.width and char != ' ':
                    if y < self.config.robot_height:
                        self.frame_buffer[y][final_x] = char


class RobotHandNode(Node):
    """ROS 2 node for robot hand simulation and visualization."""

    def __init__(self):
        super().__init__('robot_hand_node')

        # Parameters
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('velocity_decay', 0.95)
        self.declare_parameter('max_velocity', 2.0)  # units/sec

        self.update_rate = self.get_parameter('update_rate').value
        self.velocity_decay = self.get_parameter('velocity_decay').value
        self.max_velocity = self.get_parameter('max_velocity').value

        # State
        self.position = 0.0  # -1.0 to 1.0
        self.velocity = 0.0  # Current velocity
        self.target_velocity = 0.0  # Commanded velocity

        # Visualization
        self.config = TerminalConfig()
        self.visualizer = RobotHandVisualizer(self.config)

        # QoS
        qos = QoSPresetProfiles.SENSOR_DATA.value

        # Subscriber for velocity commands
        self.velocity_sub = self.create_subscription(
            RobotVelocity,
            'robot/velocity_cmd',
            self.velocity_callback,
            qos
        )

        # Publisher for position
        self.position_pub = self.create_publisher(
            RobotPosition,
            'robot/position',
            qos
        )

        # Timer for physics and visualization update
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_callback
        )

    def velocity_callback(self, msg: RobotVelocity):
        """Handle incoming velocity commands."""
        # Input: -1.0 to 1.0, representing percentage of max velocity with direction
        # Clamp to valid range and scale by max velocity
        clamped = max(-1.0, min(1.0, msg.velocity))
        self.target_velocity = clamped * self.max_velocity

    def update_callback(self):
        """Update physics and render visualization."""
        self.velocity = self.target_velocity

        # Apply velocity decay when no input
        if abs(self.target_velocity) < 0.01:
            self.velocity *= self.velocity_decay

        # Update position (velocity is in units/sec, dt is in seconds)
        dt = 1.0 / self.update_rate
        self.position += self.velocity * dt

        # Clamp position at boundaries (no bounce for smoother feel)
        if self.position > 1.0:
            self.position = 1.0
            self.velocity = 0.0
        elif self.position < -1.0:
            self.position = -1.0
            self.velocity = 0.0

        # Publish position
        pos_msg = RobotPosition()
        pos_msg.position = self.position
        self.position_pub.publish(pos_msg)

        # Render visualization
        frame = self.visualizer.render_frame(self.position, self.velocity / self.max_velocity)
        sys.stdout.write(frame)
        sys.stdout.flush()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Use alternate screen buffer to completely isolate from terminal history
    sys.stdout.write('\033[?1049h')  # Switch to alternate screen buffer
    sys.stdout.write('\033[?25l')    # Hide cursor
    sys.stdout.write('\033[2J')      # Clear alternate screen
    sys.stdout.flush()

    node = RobotHandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal state
        sys.stdout.write('\033[?25h')    # Show cursor
        sys.stdout.write('\033[?1049l')  # Switch back to main screen buffer
        sys.stdout.flush()

        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
