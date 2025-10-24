"""
Planning Configuration and Data Types
Mission phases, poses, and parameter definitions
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

# Grid value thresholds
THRESHOLD_FREE = 254
THRESHOLD_OBSTACLE = 40


class Phase(Enum):
    """Mission execution phases"""
    EXPLORING = auto()
    RETURNING = auto()
    NAVIGATING_TO_GOAL = auto()
    COMPLETED = auto()


@dataclass
class Position:
    """Spatial position with orientation (world frame, CCW positive)"""
    x_meters: float
    y_meters: float
    heading_degrees: float


@dataclass
class WheelConfig:
    """Robot wheel geometry parameters"""
    radius_mm: float
    half_baseline_mm: float
    encoder_ticks_per_revolution: int


@dataclass
class Tolerance:
    """Acceptable position and orientation errors"""
    position_tolerance_m: float = 0.07
    heading_tolerance_deg: float = 40


@dataclass
class SpeedLimits:
    """Velocity constraints (for reference, not used in direct control)"""
    max_linear_speed_mps: float = 0.25
    max_angular_speed_dps: float = 90.0
    min_motion_duration_s: float = 0.2
    max_motion_duration_s: float = 1.0