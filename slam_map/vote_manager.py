"""
Voting Mechanism for Robust Occupancy Mapping
Handles pixel locking based on vote accumulation
"""
import numpy as np
from typing import Tuple

# Constants
OBSTACLE_GRAY = 60
FREE_GRAY = 230


class VoteManager:
    """Manages voting and locking for grid cells"""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

        # Vote arrays
        self.votes_obstacle = np.zeros((height, width), dtype=np.int16)
        self.votes_free = np.zeros((height, width), dtype=np.int16)

        # Locked state
        self.locked_values = np.full((height, width), 128, dtype=np.uint8)
        self.is_locked = np.zeros((height, width), dtype=bool)

        # Thresholds
        self.config = {
            'lock_obstacle_votes': 4,
            'lock_free_votes': 10,
            'conflict_ratio': 1,
            'min_votes_filter': 2,
        }

    def reset(self):
        """Reset all votes and locks"""
        self.votes_obstacle.fill(0)
        self.votes_free.fill(0)
        self.locked_values.fill(128)
        self.is_locked.fill(False)

    def accumulate_votes(self, slam_data: np.ndarray):
        """Accumulate votes from SLAM observations"""
        obstacle_mask = (slam_data <= OBSTACLE_GRAY)
        free_mask = (slam_data >= FREE_GRAY)
        votable = ~self.is_locked

        self.votes_obstacle[obstacle_mask & votable] += 1
        self.votes_free[free_mask & votable] += 1

    def update_locks(self):
        """Update locked pixels based on accumulated votes"""
        obs_th = self.config['lock_obstacle_votes']
        free_th = self.config['lock_free_votes']
        ratio = self.config['conflict_ratio']
        min_v = self.config['min_votes_filter']

        # Process all unlocked cells
        unlocked_mask = ~self.is_locked
        y_coords, x_coords = np.where(unlocked_mask)

        for y, x in zip(y_coords, x_coords):
            obs_v = self.votes_obstacle[y, x]
            free_v = self.votes_free[y, x]

            if obs_v + free_v < min_v:
                continue

            lock_val = self._determine_lock_value(obs_v, free_v, obs_th, free_th, ratio)

            if lock_val is not None:
                self.locked_values[y, x] = lock_val
                self.is_locked[y, x] = True

    def _determine_lock_value(self, obs_votes: int, free_votes: int,
                              obs_threshold: int, free_threshold: int,
                              ratio: float) -> int:
        """Determine if cell should be locked and to what value"""

        # Strong obstacle evidence
        if obs_votes >= obs_threshold and obs_votes >= free_votes * ratio:
            return 0

        # Strong free evidence
        if free_votes >= free_threshold and free_votes >= obs_votes * ratio:
            return 255

        # Conflict resolution
        if obs_votes >= obs_threshold and free_votes >= free_threshold:
            if obs_votes > free_votes * 1.2:
                return 0
            elif free_votes > obs_votes * 1.2:
                return 255

        return None

    def merge_with_slam(self, slam_data: np.ndarray) -> np.ndarray:
        """Merge locked values with current SLAM data"""
        result = slam_data.copy()
        result[self.is_locked] = self.locked_values[self.is_locked]
        return result

    def get_statistics(self) -> dict:
        """Get voting statistics"""
        total = self.width * self.height
        locked_count = np.sum(self.is_locked)
        locked_obs = np.sum(self.is_locked & (self.locked_values <= OBSTACLE_GRAY))
        locked_free = np.sum(self.is_locked & (self.locked_values >= FREE_GRAY))

        return {
            'total': total,
            'locked': int(locked_count),
            'locked_obstacles': int(locked_obs),
            'locked_free': int(locked_free),
            'lock_percentage': float(locked_count / total * 100),
            'max_obs_votes': int(np.max(self.votes_obstacle)),
            'max_free_votes': int(np.max(self.votes_free)),
        }