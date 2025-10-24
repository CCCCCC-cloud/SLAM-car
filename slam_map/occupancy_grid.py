"""
Occupancy Grid Map with Coordinate Transformation
Manages 2D grid representation of the environment
"""
from typing import List, Tuple, Optional
import numpy as np
from slam_map.vote_manager import VoteManager
from slam_map.map_visualizer import MapVisualizer


class OccupancyGrid:
    """2D occupancy grid with voting-based updates"""

    def __init__(self, size_pixels: int, size_meters: float):
        self.size_pix = size_pixels
        self.size_m = size_meters
        self.scale = size_meters / float(size_pixels)  # meters per pixel

        # Grid data (grayscale: 0=obstacle, 128=unknown, 255=free)
        self.data: List[List[int]] = [[128] * size_pixels for _ in range(size_pixels)]

        # Voting system
        self.voter = VoteManager(size_pixels, size_pixels)

        # Robot state
        self.robot_pose = None  # (x_pix, y_pix, theta_deg)

        # Visualization
        self.renderer = MapVisualizer(
            size_pixels, size_meters,
            title="SLAM Occupancy Grid",
            show_trajectory=True,
            origin_lower_left=True
        )

    def update_from_slam(self, raw_bytes: bytearray):
        """Update grid from BreezySLAM output bytes"""
        # Reshape bytes to 2D array
        slam_array = np.reshape(
            np.frombuffer(raw_bytes, dtype=np.uint8),
            (self.size_pix, self.size_pix)
        )

        # Accumulate votes
        self.voter.accumulate_votes(slam_array)

        # Update locks
        self.voter.update_locks()

        # Merge locked pixels with SLAM data
        final_grid = self.voter.merge_with_slam(slam_array)

        # Convert to list format
        self.data = final_grid.tolist()

    # === Coordinate Transforms ===

    def to_pixels(self, x_m: float, y_m: float) -> Tuple[float, float]:
        """Convert world coordinates (meters) to pixel coordinates"""
        x_pix = x_m / self.scale
        y_pix = y_m / self.scale
        return (x_pix, y_pix)

    def to_meters(self, x_pix: float, y_pix: float) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates (meters)"""
        x_m = x_pix * self.scale
        y_m = y_pix * self.scale
        return (x_m, y_m)

    # === Robot Pose Management ===

    def update_robot_pose(self, x_m: float, y_m: float, theta_deg: float):
        """Update current robot pose"""
        x_pix, y_pix = self.to_pixels(x_m, y_m)
        self.robot_pose = (x_pix, y_pix, theta_deg)

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Get current robot pose in pixel coordinates"""
        return self.robot_pose

    # === Grid Access ===

    def get_cell(self, x_pix: int, y_pix: int) -> int:
        """Get cell value (0-255)"""
        if 0 <= x_pix < self.size_pix and 0 <= y_pix < self.size_pix:
            return self.data[y_pix][x_pix]
        return 0  # Out of bounds = obstacle

    def is_valid_position(self, x_pix: int, y_pix: int) -> bool:
        """Check if position is within grid bounds"""
        return 0 <= x_pix < self.size_pix and 0 <= y_pix < self.size_pix

    # === Statistics ===

    def get_vote_info(self) -> dict:
        """Get voting statistics"""
        return self.voter.get_statistics()