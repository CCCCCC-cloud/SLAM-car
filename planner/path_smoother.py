"""
Path Simplification and Smoothing
Reduces waypoint count while maintaining clearance
"""
from typing import List, Tuple
import math
from planner.planning_config import THRESHOLD_OBSTACLE


class PathSmoother:
    """Simplify paths by removing redundant waypoints"""

    def __init__(self,
                 obstacle_threshold: int = THRESHOLD_OBSTACLE,
                 enable_smoothing: bool = True):

        self.obstacle_th = int(obstacle_threshold)
        self.smoothing_enabled = bool(enable_smoothing)

    def simplify(self,
                 raw_path: List[Tuple[int, int]],
                 grid: List[List[int]],
                 target_spacing: int = 40,
                 min_spacing: int = 2) -> List[Tuple[int, int]]:
        """Simplify path by removing intermediate waypoints"""

        if not self.smoothing_enabled or len(raw_path) <= 2:
            return raw_path

        simplified = [raw_path[0]]
        current_idx = 0

        while current_idx < len(raw_path) - 1:
            next_idx = self._find_farthest_visible(
                raw_path, grid, current_idx,
                target_spacing, min_spacing
            )
            simplified.append(raw_path[next_idx])
            current_idx = next_idx

        # Ensure goal is included
        if simplified[-1] != raw_path[-1]:
            simplified.append(raw_path[-1])

        print(f"Path simplified: {len(raw_path)} → {len(simplified)} waypoints "
              f"({len(simplified) / len(raw_path):.1%})")

        return simplified

    def _find_farthest_visible(self,
                               path: List[Tuple[int, int]],
                               grid: List[List[int]],
                               current: int,
                               target_dist: int,
                               min_dist: int) -> int:
        """Find farthest visible waypoint from current position"""

        current_pos = path[current]
        best_idx = current + 1
        best_score = float('inf')

        # Search forward along path
        max_search = min(current + target_dist * 2, len(path) - 1)

        for candidate_idx in range(current + 1, max_search + 1):
            candidate_pos = path[candidate_idx]
            distance = self._compute_distance(current_pos, candidate_pos)

            # Skip if too close (unless at end)
            if distance < min_dist and candidate_idx < len(path) - 1:
                continue

            # Check line of sight
            if not self._has_clear_line(grid, current_pos, candidate_pos):
                continue

            # Score based on distance to target
            distance_score = abs(distance - target_dist)

            if distance_score < best_score:
                best_score = distance_score
                best_idx = candidate_idx

            # Stop if within acceptable range
            if target_dist * 0.9 <= distance <= target_dist * 1.1:
                break

        return best_idx

    def _compute_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Euclidean distance"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return math.sqrt(dx * dx + dy * dy)

    def _has_clear_line(self, grid: List[List[int]],
                        start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """Check line-of-sight using Bresenham's algorithm"""

        H, W = len(grid), len(grid[0]) if grid else 0
        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        if dx == 0 and dy == 0:
            return True

        x, y = x0, y0
        x_step = 1 if x1 > x0 else -1
        y_step = 1 if y1 > y0 else -1

        if dx > dy:
            error = dx / 2.0
            while x != x1:
                if not (0 <= x < W and 0 <= y < H) or grid[y][x] <= self.obstacle_th:
                    return False
                error -= dy
                if error < 0:
                    y += y_step
                    error += dx
                x += x_step
        else:
            error = dy / 2.0
            while y != y1:
                if not (0 <= x < W and 0 <= y < H) or grid[y][x] <= self.obstacle_th:
                    return False
                error -= dx
                if error < 0:
                    x += x_step
                    error += dy
                y += y_step

        # Check final position
        if not (0 <= x1 < W and 0 <= y1 < H) or grid[y1][x1] <= self.obstacle_th:
            return False

        return True