"""
High-Level Mission Control
State machine for exploration, navigation, and goal phases
"""
from dataclasses import dataclass
from typing import Optional, Tuple, List
import math

from slam_map.occupancy_grid import OccupancyGrid
from planner.planning_config import *
from planner.transform import CoordinateTransform
from planner.frontier_detector import FrontierDetector
from planner.frontier_selector import FrontierSelector
from planner.reachability import ReachabilityChecker
from planner.region_tracker import RegionTracker
from planner.astar_planner import AStarPlanner
from planner.path_smoother import PathSmoother
from planner.command_generator import CommandGenerator


@dataclass
class StatusReport:
    """Mission status for visualization"""
    current_phase: Phase
    robot_position: Position
    target_waypoint: Optional[Position]
    detected_frontiers: int
    tracked_region: Optional[Tuple[int, int, int, int]]
    goal_discovered: bool
    path_length: int
    status_message: str = ""
    frontier_list: Optional[List[Tuple[int, int]]] = None
    selected_frontier: Optional[Tuple[int, int]] = None  # ← 修复：移除多余的]
    sampled_frontiers: Optional[List[Tuple[int, int]]] = None
    goal_position: Optional[Tuple[int, int]] = None

class MissionController:
    """High-level navigation state machine"""

    def __init__(self,
                 occupancy_grid: OccupancyGrid,
                 wheel_config: WheelConfig,
                 target_world: Optional[Tuple[float, float]] = None,
                 speed_limits: SpeedLimits = SpeedLimits(),
                 tolerance: Tolerance = Tolerance()):

        self.grid_map = occupancy_grid
        self.current_phase = Phase.EXPLORING

        # Component initialization
        self.coord_transform = CoordinateTransform(
            occupancy_grid.size_pix,
            occupancy_grid.size_pix * occupancy_grid.scale
        )

        self.region_tracker = RegionTracker()
        self.frontier_finder = FrontierDetector()
        self.frontier_chooser = FrontierSelector()
        self.goal_verifier = ReachabilityChecker()
        self.path_finder = AStarPlanner()
        self.path_optimizer = PathSmoother()
        self.cmd_gen = CommandGenerator(tolerance)

        # Mission parameters
        self.target_world_coords = target_world
        self.target_grid_coords: Optional[Tuple[int, int]] = None

        if target_world:
            self.target_grid_coords = self.coord_transform.world_to_grid(
                target_world[0], target_world[1]
            )
            print(f"Target: world {target_world} → grid {self.target_grid_coords}")

        # State tracking
        self.start_position: Optional[Position] = None
        self.exit_position: Optional[Position] = None
        self.current_path: List[Tuple[int, int]] = []
        self.active_waypoint: Optional[Position] = None
        self.frontier_candidates = []
        self.selected_frontier_target = None

        # Statistics
        self.detected_frontiers: List[Tuple[int, int]] = []
        self.known_region: Optional[Tuple[int, int, int, int]] = None
        self.message: str = ""

        self.sampled_frontier_cache = None
        self.should_reselect = False

    def record_start_position(self, position: Position):
        """Record initial position"""
        self.start_position = position

    def record_exit_position(self, position: Position):
        """Record exit position"""
        self.exit_position = position

    def compute_next_action(self, current_position: Position) -> Tuple[float, float, StatusReport]:
        """Main planning cycle"""

        self._update_world_model()
        self._ensure_active_waypoint(current_position)

        # Generate motion command
        if self.active_waypoint:
            turn_deg, distance_m = self.cmd_gen.waypoint_to_command(
                current_position, self.active_waypoint
            )
        else:
            turn_deg, distance_m = self.cmd_gen.generate_swing_command()

        # Compile status report
        report = StatusReport(
            current_phase=self.current_phase,
            robot_position=current_position,
            target_waypoint=self.active_waypoint,
            detected_frontiers=len(self.detected_frontiers),
            tracked_region=self.known_region,
            goal_discovered=(self.exit_position is not None),
            path_length=len(self.current_path),
            status_message=self.message,
            frontier_list=self.detected_frontiers.copy(),
            selected_frontier=self.selected_frontier_target,
            sampled_frontiers=self.frontier_chooser.sampled_points,
            goal_position=self.target_grid_coords
        )

        return turn_deg, distance_m, report

    def _update_world_model(self):
        """Update map understanding"""

        robot_x, robot_y, _ = self.grid_map.get_robot_pose()
        self.known_region = self.region_tracker.update_bounds(
            self.grid_map.data,
            must_include=(int(round(robot_x)), int(round(robot_y)))
        )

        if self.current_phase == Phase.EXPLORING:
            self.detected_frontiers = self.frontier_finder.find_frontiers(
                self.grid_map.data, self.known_region
            )
            self.message = "Exploring environment..."

            # Check if goal is reachable
            if self.target_grid_coords is not None:
                goal_reachable = self.goal_verifier.verify_path_exists(
                    self.grid_map.data,
                    (robot_x, robot_y),
                    self.target_grid_coords,
                    self.known_region
                )

                if goal_reachable:
                    self.current_phase = Phase.NAVIGATING_TO_GOAL
                    self.current_path.clear()
                    self.active_waypoint = None
                    self.message = "Goal accessible → Navigating"
                    return

            self.message = "Exploring toward goal..."

    def _ensure_active_waypoint(self, current_position: Position):
        """Maintain valid waypoint"""

        if self.start_position is None:
            self.start_position = Position(
                current_position.x_meters,
                current_position.y_meters,
                current_position.heading_degrees
            )

        # Keep existing waypoint if not reached
        if self.active_waypoint and not self.cmd_gen.check_reached(
                current_position, self.active_waypoint, Tolerance()
        ):
            return

        # Generate new waypoint based on phase
        if self.current_phase == Phase.EXPLORING:
            robot_x, robot_y, _ = self.grid_map.get_robot_pose()

            # Reselect if near target or path short
            if self._is_near_frontier(robot_x, robot_y, self.selected_frontier_target) or \
                    len(self.current_path) <= 3:
                self._plan_exploration_waypoint(current_position)
            else:
                self.active_waypoint = self._pop_path_waypoint(current_position)

        elif self.current_phase == Phase.RETURNING:
            if self._plan_path_to_position(current_position, self.start_position, False):
                self.active_waypoint = self._pop_path_waypoint(current_position)
            else:
                if self.cmd_gen.check_reached(current_position, self.start_position, Tolerance()):
                    if self.exit_position:
                        self.current_phase = Phase.NAVIGATING_TO_GOAL
                        self.current_path.clear()
                        self.active_waypoint = None
                        self.message = "At start → Navigate to goal"
                    else:
                        self.message = "Awaiting exit coordinates"
                else:
                    self.message = "No path to start"

        elif self.current_phase == Phase.NAVIGATING_TO_GOAL:
            goal_position = Position(self.target_world_coords[0], self.target_world_coords[1], 0)
            if self._plan_path_to_position(current_position, goal_position, True):
                self.active_waypoint = self._pop_path_waypoint(current_position)
            else:
                robot_x, robot_y, _ = self.grid_map.get_robot_pose()
                if self._is_near_frontier(robot_x, robot_y, self.target_grid_coords):
                    self.current_phase = Phase.RETURNING
                    self.active_waypoint = None
                    self.message = "Goal reached → Return home"

        elif self.current_phase == Phase.COMPLETED:
            self.active_waypoint = None
            self.message = "Mission complete"

    def _plan_exploration_waypoint(self, current_position: Position):
        """Generate waypoint for exploration"""

        if not self.detected_frontiers:
            self.message = "No frontiers available"
            self.active_waypoint = None
            self.frontier_candidates = []
            return

        robot_x, robot_y, _ = self.grid_map.get_robot_pose()

        if len(self.frontier_candidates) < 1:
            self.should_reselect = False

        if not self.should_reselect:
            candidates = self.frontier_chooser.select_candidates(
                (robot_x, robot_y),
                self.detected_frontiers,
                self.grid_map.data,
                target_pos=self.target_grid_coords,
                region=self.known_region
            )
            self.frontier_candidates = candidates
            if len(candidates) <= 0:
                return
            target = self.frontier_candidates.pop(0)
            self.selected_frontier_target = target
            self.should_reselect = True
        else:
            # 添加边界检查
            if len(self.frontier_candidates) > 0:
                target = self.frontier_candidates.pop(0)
                self.selected_frontier_target = target
            else:
                # 列表已空，强制重新选择
                self.should_reselect = False
                self.message = "Frontier list exhausted, reselecting..."
                self.active_waypoint = None
                return

        if target is None:
            self.message = "No valid frontier"
            self.active_waypoint = None
            return

        # Plan path to frontier
        self.current_path = self.path_finder.compute_path(
            self.grid_map.data,
            (int(round(robot_x)), int(round(robot_y))),
            target,
            self.known_region
        )

        if len(self.current_path) < 2:
            self.message = "Cannot reach frontier"
            self.active_waypoint = None
            self.should_reselect = True
            return
        else:
            self.should_reselect = False

    def _plan_path_to_position(self, current: Position,
                               target: Position, use_world_coords: bool) -> bool:
        """Plan path to target position"""

        robot_x, robot_y, _ = self.grid_map.get_robot_pose()

        if use_world_coords:
            goal_x, goal_y = self.coord_transform.world_to_grid(
                target.x_meters, target.y_meters
            )
        else:
            goal_x, goal_y = self.grid_map.to_pixels(
                target.x_meters, target.y_meters
            )

        self.current_path = self.path_finder.compute_path(
            self.grid_map.data,
            (int(round(robot_x)), int(round(robot_y))),
            (int(round(goal_x)), int(round(goal_y))),
            self.known_region
        )

        return len(self.current_path) >= 3

    def _pop_path_waypoint(self, current: Position) -> Optional[Position]:
        """Get next waypoint from path"""

        if len(self.current_path) < 2:
            return None

        next_point = self.current_path.pop(1)
        return self._grid_to_waypoint(current, next_point)

    def _grid_to_waypoint(self, current: Position,
                          grid_point: Tuple[int, int]) -> Position:
        """Convert grid coordinate to waypoint"""

        x_m, y_m = self.grid_map.to_meters(grid_point[0], grid_point[1])
        dx, dy = x_m - current.x_meters, y_m - current.y_meters
        heading = math.degrees(math.atan2(dy, dx))

        return Position(x_m, y_m, heading)

    @staticmethod
    def _is_near_frontier(robot_x: float, robot_y: float,
                          frontier: Optional[Tuple[int, int]],
                          threshold: float = 20) -> bool:
        """Check if near frontier point"""

        if frontier is None:
            return False

        fx, fy = frontier
        distance = math.sqrt((robot_x - fx) ** 2 + (robot_y - fy) ** 2)
        return distance < threshold