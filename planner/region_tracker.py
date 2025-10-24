"""
Region of Interest Tracker
Maintains and updates known maze boundaries
"""
from typing import List, Tuple, Optional


class RegionTracker:
    """Track known map region boundaries"""

    def __init__(self,
                 boundary_margin: int = 20,
                 min_region_width: int = 5,
                 min_region_height: int = 5,
                 min_region_area: int = 400,
                 allow_shrinkage: bool = False,
                 max_shrink_step: int = 4):

        self.current_bounds: Optional[Tuple[int, int, int, int]] = None
        self.margin = int(boundary_margin)
        self.min_w = int(min_region_width)
        self.min_h = int(min_region_height)
        self.min_area = int(min_region_area)
        self.shrinkage_allowed = bool(allow_shrinkage)
        self.max_shrink = int(max_shrink_step)

    def update_bounds(self, grid: List[List[int]],
                     must_include: Optional[Tuple[int, int]] = None) -> Optional[Tuple[int, int, int, int]]:
        """Update region bounds based on known cells"""

        if not grid or not grid[0]:
            return self.current_bounds

        H, W = len(grid), len(grid[0])

        # Find extent of known cells
        min_x, min_y = W, H
        max_x, max_y = -1, -1
        known_cell_count = 0

        for row_idx in range(H):
            row = grid[row_idx]
            for col_idx in range(W):
                if row[col_idx] <= 20:  # Known cell (obstacle or free)
                    known_cell_count += 1
                    min_x = min(min_x, col_idx)
                    max_x = max(max_x, col_idx)
                    min_y = min(min_y, row_idx)
                    max_y = max(max_y, row_idx)

        if known_cell_count == 0:
            return self.current_bounds

        if max_x < min_x or max_y < min_y:
            return self.current_bounds

        # Check minimum size requirements
        width = max_x - min_x + 1
        height = max_y - min_y + 1
        if width < self.min_w or height < self.min_h or (width * height) < self.min_area:
            return self.current_bounds

        # Apply inward margin (shrink bounds)
        min_x_margin = max(0, min_x + 4)
        min_y_margin = max(0, min_y + 4)
        max_x_margin = min(W - 1, max_x - 8)
        max_y_margin = min(H - 1, max_y - 8)

        # Ensure must_include point is within bounds
        if must_include is not None:
            include_x, include_y = must_include
            min_x_margin = min(min_x_margin, max(0, include_x))
            min_y_margin = min(min_y_margin, max(0, include_y))
            max_x_margin = max(max_x_margin, min(W - 1, include_x))
            max_y_margin = max(max_y_margin, min(H - 1, include_y))

        candidate_bounds = (min_x_margin, min_y_margin, max_x_margin, max_y_margin)

        # Initialize if first update
        if self.current_bounds is None:
            self.current_bounds = candidate_bounds
            return self.current_bounds

        # Merge with existing bounds
        curr_x0, curr_y0, curr_x1, curr_y1 = self.current_bounds
        cand_x0, cand_y0, cand_x1, cand_y1 = candidate_bounds

        # Expand only (or controlled shrink)
        new_x0 = min(curr_x0, cand_x0)
        new_y0 = min(curr_y0, cand_y0)
        new_x1 = max(curr_x1, cand_x1)
        new_y1 = max(curr_y1, cand_y1)

        if self.shrinkage_allowed:
            new_x0 = self._controlled_shrink(curr_x0, cand_x0, inward=True)
            new_y0 = self._controlled_shrink(curr_y0, cand_y0, inward=True)
            new_x1 = self._controlled_shrink(curr_x1, cand_x1, inward=False)
            new_y1 = self._controlled_shrink(curr_y1, cand_y1, inward=False)

            # Validate size after shrink
            if (new_x1 - new_x0 + 1) < self.min_w or (new_y1 - new_y0 + 1) < self.min_h:
                new_x0, new_y0, new_x1, new_y1 = curr_x0, curr_y0, curr_x1, curr_y1

        self.current_bounds = (new_x0, new_y0, new_x1, new_y1)
        return self.current_bounds

    def _controlled_shrink(self, prev_edge: int, cand_edge: int, inward: bool) -> int:
        """Limit shrinkage per update"""
        if inward:
            # Left/top edge moving right/down
            if cand_edge <= prev_edge:
                return cand_edge
            else:
                return min(prev_edge + self.max_shrink, cand_edge)
        else:
            # Right/bottom edge moving left/up
            if cand_edge >= prev_edge:
                return cand_edge
            else:
                return max(prev_edge - self.max_shrink, cand_edge)

    def contains_point(self, x_pix: float, y_pix: float) -> bool:
        """Check if point is inside tracked region"""
        if not self.current_bounds:
            return True
        x0, y0, x1, y1 = self.current_bounds
        return (x0 <= x_pix <= x1) and (y0 <= y_pix <= y1)

    def get_bounds(self) -> Optional[Tuple[int, int, int, int]]:
        """Get current region bounds (x0, y0, x1, y1)"""
        return self.current_bounds