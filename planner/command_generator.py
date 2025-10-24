"""
Motion Command Generation
Converts waypoints to turn and drive commands
"""
from typing import Tuple
import math
from planner.planning_config import Position, Tolerance


class CommandGenerator:
    """Generate turn and straight motion commands"""

    def __init__(self, tolerance: Tolerance = Tolerance()):
        self.tol = tolerance
        self._swing_direction = 1

    def waypoint_to_command(self,
                            current: Position,
                            target: Position) -> Tuple[float, float]:
        """Convert waypoint to (turn_deg, distance_m)"""

        dx = target.x_meters - current.x_meters
        dy = target.y_meters - current.y_meters
        distance = math.hypot(dx, dy)
        target_heading = math.degrees(math.atan2(dy, dx))

        # If close enough, handle orientation
        if distance <= self.tol.position_tolerance_m:
            heading_error = self._wrap_angle(target.heading_degrees - current.heading_degrees)
            if abs(heading_error) < 3:
                return 0.0, 0.0
            else:
                return heading_error, 0.0

        # Otherwise, turn to face waypoint then drive
        heading_error = self._wrap_angle(target_heading - current.heading_degrees)
        if abs(heading_error) < 2:
            return 0.0, distance
        else:
            return heading_error, distance

    def generate_swing_command(self, swing_angle_deg: float = 15.0) -> Tuple[float, float]:
        """Generate in-place swing motion for scanning"""

        turn = swing_angle_deg * self._swing_direction
        self._swing_direction *= -1  # Alternate direction

        distance = 0.05  # Small forward motion
        return turn, distance

    @staticmethod
    def _wrap_angle(angle_deg: float) -> float:
        """Normalize angle to [-180, 180)"""
        wrapped = (angle_deg + 180.0) % 360.0 - 180.0
        return 180.0 if wrapped == -180.0 else wrapped

    @staticmethod
    def check_reached(current: Position, goal: Position, tol: Tolerance) -> bool:
        """Check if goal position is reached"""
        distance = math.hypot(
            current.x_meters - goal.x_meters,
            current.y_meters - goal.y_meters
        )
        return distance <= tol.position_tolerance_m