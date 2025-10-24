"""
Drawing Utility Functions
Helper functions for visualization rendering
"""
import math
from typing import Tuple, List


class DrawingTools:
    """Collection of drawing helper functions"""

    @staticmethod
    def compute_arrow_points(x: float, y: float, theta_deg: float,
                             shaft_len: float, head_len: float,
                             head_width: float) -> dict:
        """Compute arrow geometry for robot visualization"""
        theta_rad = math.radians(theta_deg)
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)

        # Shaft endpoint
        dx_shaft = shaft_len * cos_t
        dy_shaft = shaft_len * sin_t

        return {
            'base': (x, y),
            'shaft_end': (x + dx_shaft, y + dy_shaft),
            'dx': dx_shaft,
            'dy': dy_shaft,
            'head_w': head_width,
            'head_l': head_len,
        }

    @staticmethod
    def pixel_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points"""
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def interpolate_color(value: float, min_val: float, max_val: float,
                          color_a: Tuple, color_b: Tuple) -> Tuple:
        """Linear color interpolation"""
        t = (value - min_val) / (max_val - min_val) if max_val != min_val else 0
        t = max(0, min(1, t))

        r = color_a[0] * (1 - t) + color_b[0] * t
        g = color_a[1] * (1 - t) + color_b[1] * t
        b = color_a[2] * (1 - t) + color_b[2] * t

        return (int(r), int(g), int(b))

    @staticmethod
    def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x, y = x0, y0
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1

        if dx > dy:
            error = dx / 2.0
            while x != x1:
                points.append((x, y))
                error -= dy
                if error < 0:
                    y += y_inc
                    error += dx
                x += x_inc
        else:
            error = dy / 2.0
            while y != y1:
                points.append((x, y))
                error -= dx
                if error < 0:
                    x += x_inc
                    error += dy
                y += y_inc

        points.append((x1, y1))
        return points

    @staticmethod
    def format_pose_label(x_m: float, y_m: float, theta_deg: float) -> str:
        """Format pose information for display"""
        return f"({x_m:.2f}m, {y_m:.2f}m) θ={theta_deg:.1f}°"

    @staticmethod
    def rect_from_bounds(x0: int, y0: int, x1: int, y1: int,
                         sx: float, sy: float, origin: str) -> Tuple:
        """Calculate rectangle parameters for matplotlib"""
        if origin == 'lower':
            x_m = x0 * sx
            y_m = y0 * sy
            w_m = (x1 - x0 + 1) * sx
            h_m = (y1 - y0 + 1) * sy
        else:
            x_m = x0 * sx
            y_m = -(y1 + 1) * sy
            w_m = (x1 - x0 + 1) * sx
            h_m = (y1 - y0 + 1) * sy

        return (x_m, y_m, w_m, h_m)