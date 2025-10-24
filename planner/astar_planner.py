"""
A* Pathfinding Algorithm
Grid-based optimal path search with obstacle inflation
"""
from typing import List, Tuple, Optional, Dict
import math
import heapq
from planner.planning_config import THRESHOLD_FREE, THRESHOLD_OBSTACLE


class AStarPlanner:
    """A* path planner with configurable heuristics"""

    def __init__(self,
                 diagonal_movement: bool = True,
                 inflation_radius: int = 25,
                 free_threshold: int = THRESHOLD_FREE,
                 obstacle_threshold: int = THRESHOLD_OBSTACLE,
                 snap_distance: int = 8):

        self.allow_diagonal = bool(diagonal_movement)
        self.inflate_radius = int(max(0, inflation_radius))
        self.free_th = int(free_threshold)
        self.obstacle_th = int(obstacle_threshold)
        self.snap_radius = int(max(0, snap_distance))

    def compute_path(self,
                     grid: List[List[int]],
                     start_pos: Tuple[int, int],
                     goal_pos: Tuple[int, int],
                     region_bounds: Optional[Tuple[int, int, int, int]]) -> List[Tuple[int, int]]:
        """Compute optimal path using A*"""

        if not grid or not grid[0]:
            return []

        H, W = len(grid), len(grid[0])

        # Define search region
        if region_bounds is None:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        else:
            x0, y0, x1, y1 = region_bounds
            x0 = max(0, min(x0, W - 1))
            x1 = max(0, min(x1, W - 1))
            y0 = max(0, min(y0, H - 1))
            y1 = max(0, min(y1, H - 1))
            if x1 < x0 or y1 < y0:
                return []

        # Build traversability map with inflation
        traversable, region_w, region_h = self._build_traversable_map(
            grid, (x0, y0, x1, y1)
        )

        # Convert to local coordinates
        start_x, start_y = start_pos
        goal_x, goal_y = goal_pos

        if not (x0 <= start_x <= x1 and y0 <= start_y <= y1 and
                x0 <= goal_x <= x1 and y0 <= goal_y <= y1):
            return []

        local_start_x = start_x - x0
        local_start_y = start_y - y0
        local_goal_x = goal_x - x0
        local_goal_y = goal_y - y0

        if not self._in_bounds(local_start_x, local_start_y, region_w, region_h):
            return []
        if not self._in_bounds(local_goal_x, local_goal_y, region_w, region_h):
            return []

        # Snap to traversable cells if needed
        if not traversable[local_start_y][local_start_x]:
            snapped = self._snap_to_traversable(traversable, local_start_x, local_start_y)
            if snapped is None:
                return []
            local_start_x, local_start_y = snapped

        if not traversable[local_goal_y][local_goal_x]:
            snapped = self._snap_to_traversable(traversable, local_goal_x, local_goal_y)
            if snapped is None:
                return []
            local_goal_x, local_goal_y = snapped

        if (local_start_x, local_start_y) == (local_goal_x, local_goal_y):
            return [(local_start_x + x0, local_start_y + y0)]

        # Run A*
        local_path = self._astar_search(
            traversable,
            (local_start_x, local_start_y),
            (local_goal_x, local_goal_y)
        )

        if not local_path:
            return []

        # Convert back to global coordinates
        return [(x + x0, y + y0) for (x, y) in local_path]

    def _astar_search(self,
                      traversable: List[List[bool]],
                      start: Tuple[int, int],
                      goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Core A* algorithm"""

        H, W = len(traversable), len(traversable[0]) if traversable else 0
        if H == 0 or W == 0:
            return []

        start_x, start_y = start
        goal_x, goal_y = goal

        # Movement directions and costs
        if self.allow_diagonal:
            movements = [
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2))
            ]
        else:
            movements = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

        # Heuristic function
        def heuristic(x: int, y: int) -> float:
            dx = abs(x - goal_x)
            dy = abs(y - goal_y)
            if self.allow_diagonal:
                D = 1.0
                D2 = math.sqrt(2)
                return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
            else:
                return dx + dy

        # A* data structures
        open_heap = []
        tie_breaker = 0

        cost_so_far = {(start_x, start_y): 0.0}
        parent_map: Dict[Tuple[int, int], Tuple[int, int]] = {}

        heapq.heappush(open_heap, (heuristic(start_x, start_y), tie_breaker, start_x, start_y))
        tie_breaker += 1

        closed_set = set()

        while open_heap:
            _, _, current_x, current_y = heapq.heappop(open_heap)

            if (current_x, current_y) in closed_set:
                continue

            if (current_x, current_y) == (goal_x, goal_y):
                return self._reconstruct_path(parent_map, (goal_x, goal_y), (start_x, start_y))

            closed_set.add((current_x, current_y))

            # Expand neighbors
            for dx, dy, move_cost in movements:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy

                if not self._in_bounds(neighbor_x, neighbor_y, W, H):
                    continue

                if not traversable[neighbor_y][neighbor_x]:
                    continue

                # Prevent diagonal corner-cutting
                if self.allow_diagonal and dx != 0 and dy != 0:
                    adj1_x, adj1_y = current_x + dx, current_y
                    adj2_x, adj2_y = current_x, current_y + dy
                    if not (traversable[adj1_y][adj1_x] and traversable[adj2_y][adj2_x]):
                        continue

                tentative_cost = cost_so_far[(current_x, current_y)] + move_cost

                if (neighbor_x, neighbor_y) not in cost_so_far or \
                        tentative_cost < cost_so_far[(neighbor_x, neighbor_y)]:
                    cost_so_far[(neighbor_x, neighbor_y)] = tentative_cost
                    parent_map[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    priority = tentative_cost + heuristic(neighbor_x, neighbor_y)
                    heapq.heappush(open_heap, (priority, tie_breaker, neighbor_x, neighbor_y))
                    tie_breaker += 1

        return []

    def _build_traversable_map(self,
                               grid: List[List[int]],
                               region: Tuple[int, int, int, int]) -> Tuple[List[List[bool]], int, int]:
        """Build binary traversability map with obstacle inflation"""

        x0, y0, x1, y1 = region
        H, W = len(grid), len(grid[0])
        region_w = x1 - x0 + 1
        region_h = y1 - y0 + 1

        traversable = [[False] * region_w for _ in range(region_h)]
        obstacles = []

        # Extract free/obstacle cells
        for y in range(y0, y1 + 1):
            row = grid[y]
            local_y = y - y0
            for x in range(x0, x1 + 1):
                cell_value = row[x]
                local_x = x - x0

                if cell_value >= self.free_th:
                    traversable[local_y][local_x] = True

                if cell_value <= self.obstacle_th:
                    obstacles.append((local_x, local_y))

        # Inflate obstacles
        r = self.inflate_radius
        if r > 0 and obstacles:
            r_squared = r * r
            for (obs_x, obs_y) in obstacles:
                x_min = max(0, obs_x - r)
                x_max = min(region_w - 1, obs_x + r)
                y_min = max(0, obs_y - r)
                y_max = min(region_h - 1, obs_y + r)

                for py in range(y_min, y_max + 1):
                    dy = py - obs_y
                    dy_sq = dy * dy
                    for px in range(x_min, x_max + 1):
                        dx = px - obs_x
                        if dx * dx + dy_sq <= r_squared:
                            traversable[py][px] = False

        return traversable, region_w, region_h

    def _snap_to_traversable(self,
                             traversable: List[List[bool]],
                             x: int, y: int) -> Optional[Tuple[int, int]]:
        """Find nearest traversable cell"""

        from collections import deque

        H, W = len(traversable), len(traversable[0]) if traversable else 0

        if self._in_bounds(x, y, W, H) and traversable[y][x]:
            return (x, y)

        # BFS for nearest traversable cell
        queue = deque()
        visited = set()
        queue.append((x, y, 0))
        visited.add((x, y))

        # Generate nearby directions
        directions = []
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                if dx == 0 and dy == 0:
                    continue
                if max(abs(dx), abs(dy)) <= 3:
                    directions.append((dx, dy))

        while queue:
            curr_x, curr_y, dist = queue.popleft()
            if dist > self.snap_radius:
                break

            if self._in_bounds(curr_x, curr_y, W, H) and traversable[curr_y][curr_x]:
                return (curr_x, curr_y)

            next_dist = dist + 1
            for dx, dy in directions:
                next_x, next_y = curr_x + dx, curr_y + dy
                if not self._in_bounds(next_x, next_y, W, H):
                    continue
                if (next_x, next_y) in visited:
                    continue
                visited.add((next_x, next_y))
                queue.append((next_x, next_y, next_dist))

        return None

    @staticmethod
    def _reconstruct_path(parent: Dict[Tuple[int, int], Tuple[int, int]],
                          goal: Tuple[int, int],
                          start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from parent pointers"""
        path = [goal]
        current = goal
        while current != start:
            current = parent.get(current)
            if current is None:
                return []
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _in_bounds(x: int, y: int, W: int, H: int) -> bool:
        """Check if position is within bounds"""
        return 0 <= x < W and 0 <= y < H