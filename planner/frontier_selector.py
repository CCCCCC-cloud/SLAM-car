"""
Frontier Point Selection and Scoring
Selects best frontier for exploration based on multiple criteria
"""
from typing import List, Tuple, Optional
import math
import time
from collections import deque


class FrontierSelector:
    """Select optimal frontier points for exploration"""

    def __init__(self,
                 min_range_pix: int = 30,
                 max_range_pix: int = 200,
                 spatial_grid_size: int = 10,
                 goal_weight: float = 0.6):

        self.range_min = int(min_range_pix)
        self.range_max = int(max_range_pix)
        self.grid_size = int(spatial_grid_size)
        self.goal_weight = goal_weight

        # Cache
        self.sampled_points = None

    def select_candidates(self,
                          robot_pos: Tuple[float, float],
                          all_frontiers: List[Tuple[int, int]],
                          grid_data: List[List[int]],
                          target_pos: Optional[Tuple[int, int]] = None,
                          region: Optional[Tuple[int, int, int, int]] = None) -> List[Tuple[int, int]]:
        """Main selection pipeline"""

        if not all_frontiers:
            return []

        t0 = time.time()
        rx, ry = int(robot_pos[0]), int(robot_pos[1])

        # Step 1: Distance filtering
        range_filtered = self._apply_distance_filter(all_frontiers, (rx, ry))
        if not range_filtered:
            return []

        # Step 2: Spatial downsampling
        sampled = self._spatial_downsample(range_filtered)

        # Step 3: Connectivity check
        connected = self._check_connectivity(sampled, (rx, ry), grid_data, region)
        if not connected:
            return []

        # Step 4: Goal-biased scoring
        if target_pos is not None:
            ranked = self._rank_by_goal_proximity(connected, (rx, ry), target_pos)
        else:
            ranked = connected

        self.sampled_points = connected

        elapsed = time.time() - t0
        print(f"Frontier selection: {elapsed:.3f}s | "
              f"{len(all_frontiers)}→{len(range_filtered)}→{len(sampled)}→"
              f"{len(connected)}→{len(ranked)}")

        return ranked

    def _apply_distance_filter(self, points: List[Tuple[int, int]],
                               robot: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Keep only points within distance range"""
        rx, ry = robot
        min_r2 = self.range_min ** 2
        max_r2 = self.range_max ** 2

        result = []
        for px, py in points:
            dist2 = (px - rx) ** 2 + (py - ry) ** 2
            if min_r2 <= dist2 <= max_r2:
                result.append((px, py))
        return result

    def _spatial_downsample(self, points: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Reduce density using grid-based sampling"""
        if len(points) <= 50:
            return points

        grid_buckets = {}
        for px, py in points:
            key = (px // self.grid_size, py // self.grid_size)
            if key not in grid_buckets:
                grid_buckets[key] = []
            grid_buckets[key].append((px, py))

        return [bucket[0] for bucket in grid_buckets.values()]

    def _check_connectivity(self, points: List[Tuple[int, int]],
                            robot: Tuple[int, int],
                            grid: List[List[int]],
                            region: Optional[Tuple[int, int, int, int]]) -> List[Tuple[int, int]]:
        """Filter points reachable via BFS"""

        W, H = len(grid[0]), len(grid)
        FREE_THRESH = 254

        if region is None:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        else:
            x0, y0, x1, y1 = region

        def is_robust_free(x: int, y: int) -> bool:
            """Check if cell and neighborhood are mostly free"""
            free_count = 0
            total = 0
            for dy in range(-5, 6):
                for dx in range(-5, 6):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H and x0 <= nx <= x1 and y0 <= ny <= y1:
                        total += 1
                        if grid[ny][nx] >= FREE_THRESH:
                            free_count += 1
            return total > 0 and free_count / total > 0.5

        # Find valid start point
        sx, sy = int(robot[0]), int(robot[1])
        if not (0 <= sx < W and 0 <= sy < H) or not is_robust_free(sx, sy):
            # Search nearby
            found = False
            for radius in range(1, 8):
                if found:
                    break
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        nx, ny = sx + dx, sy + dy
                        if 0 <= nx < W and 0 <= ny < H and is_robust_free(nx, ny):
                            sx, sy = nx, ny
                            found = True
                            break
            if not found:
                return points[:2] if points else []

        # BFS expansion
        reachable = set()
        queue = deque([(sx, sy)])
        reachable.add((sx, sy))

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (1, -1), (-1, 1), (1, 1)]

        while queue:
            x, y = queue.popleft()
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (nx, ny) in reachable:
                    continue
                if not (0 <= nx < W and 0 <= ny < H and x0 <= nx <= x1 and y0 <= ny <= y1):
                    continue
                if grid[ny][nx] < FREE_THRESH:
                    continue
                if not is_robust_free(nx, ny):
                    continue
                reachable.add((nx, ny))
                queue.append((nx, ny))

        qualified = [p for p in points if p in reachable]
        return qualified if qualified else points[:2]

    def _rank_by_goal_proximity(self, points: List[Tuple[int, int]],
                                robot: Tuple[int, int],
                                goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Score and sort points by goal proximity"""

        if not points:
            return []

        rx, ry = robot
        gx, gy = goal

        goal_vec = (gx - rx, gy - ry)
        goal_dist = math.sqrt(goal_vec[0] ** 2 + goal_vec[1] ** 2)

        if goal_dist < 1e-6:
            return points

        goal_dir = (goal_vec[0] / goal_dist, goal_vec[1] / goal_dist)

        scored = []
        for px, py in points:
            dist_to_goal = math.sqrt((px - gx) ** 2 + (py - gy) ** 2)

            cand_vec = (px - rx, py - ry)
            cand_dist = math.sqrt(cand_vec[0] ** 2 + cand_vec[1] ** 2)

            if cand_dist < 1e-6:
                continue

            alignment = (cand_vec[0] * goal_dir[0] + cand_vec[1] * goal_dir[1]) / cand_dist

            dist_score = 1.0 / (1.0 + dist_to_goal / 100.0)
            align_score = (alignment + 1.0) / 2.0

            final_score = (self.goal_weight * dist_score +
                           (1 - self.goal_weight) * align_score)

            scored.append(((px, py), final_score))

        scored.sort(key=lambda item: -item[1])
        return [coord for coord, _ in scored]