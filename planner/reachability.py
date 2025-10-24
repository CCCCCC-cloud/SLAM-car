"""
Goal Reachability Verification
Checks if target position is reachable from current location
"""
from typing import List, Tuple, Optional
from collections import deque
from planner.planning_config import THRESHOLD_FREE, THRESHOLD_OBSTACLE


class ReachabilityChecker:
    """Verify path existence to goal using BFS"""

    def __init__(self,
                 free_threshold: int = THRESHOLD_FREE,
                 unknown_value: int = 128,
                 obstacle_threshold: int = THRESHOLD_OBSTACLE,
                 goal_clearance_radius: int = 5):

        self.free_th = int(free_threshold)
        self.unknown_val = int(unknown_value)
        self.obstacle_th = int(obstacle_threshold)
        self.goal_radius = int(goal_clearance_radius)

        self._check_counter = 0
        self._cached_result = False

    def verify_path_exists(self,
                          grid: List[List[int]],
                          start_pos: Tuple[int, int],
                          goal_pos: Tuple[int, int],
                          search_region: Optional[Tuple[int, int, int, int]] = None,
                          force_check: bool = True) -> bool:
        """Check if path exists from start to goal"""

        self._check_counter += 1

        # Throttle checks (every 5th frame unless forced)
        if not force_check and self._check_counter % 5 != 0:
            return self._cached_result

        H, W = len(grid), len(grid[0]) if len(grid) > 0 else 0
        if W == 0 or H == 0:
            return False

        goal_x, goal_y = int(round(goal_pos[0])), int(round(goal_pos[1]))

        # Validate goal position
        if not (0 <= goal_x < W and 0 <= goal_y < H):
            print(f"Goal ({goal_x},{goal_y}) out of bounds")
            self._cached_result = False
            return False

        # Check goal area is traversable
        if not self._is_goal_area_clear(grid, goal_x, goal_y):
            print("Goal area blocked")
            self._cached_result = False
            return False

        # BFS path search
        start_x, start_y = int(round(start_pos[0])), int(round(start_pos[1]))
        path_exists = self._breadth_first_search(
            grid, (start_x, start_y), (goal_x, goal_y), search_region
        )

        if path_exists:
            print(f"✓ Path verified: ({start_x},{start_y}) → ({goal_x},{goal_y})")
        else:
            print("✗ No path to goal")

        self._cached_result = path_exists
        return path_exists

    def _is_goal_area_clear(self, grid: List[List[int]],
                           goal_x: int, goal_y: int) -> bool:
        """Verify goal and surrounding area are free"""

        H, W = len(grid), len(grid[0])

        # Check center cell
        if not (0 <= goal_x < W and 0 <= goal_y < H):
            return False

        if grid[goal_y][goal_x] < self.free_th:
            return False

        # Check circular region
        r = self.goal_radius
        free_cells = 0
        total_cells = 0

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue

                nx, ny = goal_x + dx, goal_y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    total_cells += 1
                    if grid[ny][nx] >= self.free_th:
                        free_cells += 1

        if total_cells == 0:
            return False

        return free_cells / total_cells >= 0.7

    def _breadth_first_search(self, grid: List[List[int]],
                             start: Tuple[int, int],
                             goal: Tuple[int, int],
                             region: Optional[Tuple[int, int, int, int]]) -> bool:
        """BFS pathfinding to check connectivity"""

        H, W = len(grid), len(grid[0])

        # Define search boundaries
        if region:
            x0, y0, x1, y1 = region
            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(W - 1, x1)
            y1 = min(H - 1, y1)
        else:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1

        start_x, start_y = start
        goal_x, goal_y = goal

        # Validate start and goal are in region
        if not (x0 <= start_x <= x1 and y0 <= start_y <= y1):
            return False
        if not (x0 <= goal_x <= x1 and y0 <= goal_y <= y1):
            return False

        # BFS initialization
        queue = deque([(start_x, start_y)])
        visited = {(start_x, start_y)}
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        max_iterations = 5000
        iteration_count = 0

        while queue and iteration_count < max_iterations:
            iteration_count += 1
            current_x, current_y = queue.popleft()

            # Check if reached goal vicinity
            if abs(current_x - goal_x) <= 8 and abs(current_y - goal_y) <= 8:
                return True

            # Expand neighbors
            for dx, dy in directions:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy

                if (neighbor_x, neighbor_y) in visited:
                    continue

                if not (x0 <= neighbor_x <= x1 and y0 <= neighbor_y <= y1):
                    continue

                if grid[neighbor_y][neighbor_x] < self.free_th:
                    continue

                visited.add((neighbor_x, neighbor_y))
                queue.append((neighbor_x, neighbor_y))

        return False