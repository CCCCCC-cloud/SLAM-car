"""
Frontier Detection Algorithm
Identifies frontier points between known free space and unknown areas
"""
from typing import List, Tuple, Optional
from planner.planning_config import THRESHOLD_FREE, THRESHOLD_OBSTACLE


class FrontierDetector:
    """Detect frontier cells in occupancy grid"""

    def __init__(self,
                 free_threshold: int = THRESHOLD_FREE,
                 unknown_value: int = 128,
                 obstacle_threshold: int = THRESHOLD_OBSTACLE,
                 safety_clearance_pix: int = 10):

        self.free_th = int(free_threshold)
        self.unknown_val = int(unknown_value)
        self.obstacle_th = int(obstacle_threshold)
        self.safety_radius = int(safety_clearance_pix)

    def find_frontiers(self, grid: List[List[int]],
                       search_region: Optional[Tuple[int, int, int, int]]) -> List[Tuple[int, int]]:
        """Detect all frontier points in the grid"""

        H = len(grid)
        W = len(grid[0]) if H else 0
        if H == 0 or W == 0:
            return []

        # Define search bounds
        if search_region is None:
            x_min, y_min, x_max, y_max = 0, 0, W - 1, H - 1
        else:
            x_min, y_min, x_max, y_max = search_region
            x_min = max(0, x_min)
            y_min = max(0, y_min)
            x_max = min(W - 1, x_max)
            y_max = min(H - 1, y_max)

        frontiers = []

        # Scan grid for frontier candidates
        for row_idx in range(max(y_min, 1), min(y_max, H - 2) + 1):
            grid_row = grid[row_idx]
            for col_idx in range(max(x_min, 1), min(x_max, W - 2) + 1):
                cell_value = grid_row[col_idx]

                # Must be free space
                if cell_value < self.free_th:
                    continue

                # Must have unknown neighbor
                if not self._has_unknown_adjacent(grid, col_idx, row_idx, W, H):
                    continue

                # Must be safe from obstacles
                if not self._is_obstacle_free_region(grid, col_idx, row_idx, W, H):
                    continue

                frontiers.append((col_idx, row_idx))

        return frontiers

    def _has_unknown_adjacent(self, grid: List[List[int]],
                              x: int, y: int, W: int, H: int) -> bool:
        """Check if cell has unknown neighbors (8-connectivity)"""
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    neighbor_val = grid[ny][nx]
                    if self.obstacle_th < neighbor_val < self.free_th:
                        return True
        return False

    def _is_obstacle_free_region(self, grid: List[List[int]],
                                 x: int, y: int, W: int, H: int) -> bool:
        """Check if circular region around point is obstacle-free"""
        r = self.safety_radius
        r_squared = r * r

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                # Check if within circular radius
                if dx * dx + dy * dy > r_squared:
                    continue

                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    if grid[ny][nx] <= self.obstacle_th:
                        return False

        return True