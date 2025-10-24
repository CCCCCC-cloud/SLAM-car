"""
Coordinate System Transformation
Converts between world (meters) and grid (pixels) coordinates
"""
from typing import Tuple


class CoordinateTransform:
    """Bidirectional coordinate transformation"""

    def __init__(self, grid_size_pixels: int, world_size_meters: float):
        self.grid_pixels = grid_size_pixels
        self.world_meters = world_size_meters
        self.scale_factor = world_size_meters / grid_size_pixels

        # Grid center as origin
        self.center_x = grid_size_pixels / 2.0
        self.center_y = grid_size_pixels / 2.0

    def world_to_grid(self, x_world: float, y_world: float) -> Tuple[int, int]:
        """Transform world coordinates to grid indices"""
        x_grid = self.center_x + (x_world / self.scale_factor)
        y_grid = self.center_y + (y_world / self.scale_factor)
        return (int(x_grid), int(y_grid))

    def grid_to_world(self, x_grid: float, y_grid: float) -> Tuple[float, float]:
        """Transform grid indices to world coordinates"""
        x_world = (x_grid - self.center_x) * self.scale_factor
        y_world = (y_grid - self.center_y) * self.scale_factor
        return (x_world, y_world)

    def is_valid_grid_position(self, x_grid: float, y_grid: float) -> bool:
        """Check if grid position is within bounds"""
        return (0 <= x_grid < self.grid_pixels and
                0 <= y_grid < self.grid_pixels)

    def clamp_to_grid(self, x_grid: float, y_grid: float) -> Tuple[int, int]:
        """Clamp coordinates to valid grid range"""
        x_clamped = max(0, min(self.grid_pixels - 1, int(x_grid)))
        y_clamped = max(0, min(self.grid_pixels - 1, int(y_grid)))
        return (x_clamped, y_clamped)