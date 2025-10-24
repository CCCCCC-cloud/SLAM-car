"""
Enhanced Map Visualization with Matplotlib
Displays grid map, robot pose, trajectory, frontiers, and ROI with improved UI
"""
from typing import List, Tuple, Optional
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.lines as mlines
import matplotlib.patches as mpatches


class MapVisualizer:
    """Real-time map visualization with enhanced UI"""

    ROBOT_HEAD_LEN_M = 0.09
    ROBOT_HEAD_W_M = 0.06
    ROBOT_SHAFT_LEN_M = 0.05

    def __init__(self, map_size_pixels: int, map_size_meters: float,
                 title: str = "SLAM Map", show_trajectory: bool = True,
                 origin_lower_left: bool = True):
        self.size_pixels = map_size_pixels
        self.size_meters = float(map_size_meters)
        self.meters_per_pixel = self.size_meters / float(self.size_pixels)
        self.title = title
        self.show_trajectory = show_trajectory
        self.origin = "lower" if origin_lower_left else "upper"

        # Create figure with improved styling
        self.fig = plt.figure(figsize=(10, 9))
        self.fig.patch.set_facecolor('#f0f0f0')

        # Main map axes
        self.ax = self.fig.add_subplot(111)

        try:
            self.fig.canvas.manager.set_window_title("🤖 SLAM Robot Visualization")
        except:
            pass

        # Styling
        self.ax.set_title(self.title, fontsize=14, fontweight='bold', pad=15)
        self.ax.set_xlabel("X Position (meters)", fontsize=11, fontweight='bold')
        self.ax.set_ylabel("Y Position (meters)", fontsize=11, fontweight='bold')
        self.ax.set_aspect("equal")
        self.ax.set_facecolor('#ffffff')
        self.ax.grid(True, alpha=0.2, linestyle='--', linewidth=0.5)

        self.extent_m = (0.0, self.size_meters, 0.0, self.size_meters)
        self.ax.set_xlim(self.extent_m[0], self.extent_m[1])
        self.ax.set_ylim(self.extent_m[2], self.extent_m[3])

        # Artists (reusable)
        self.img_artist = None
        self.car_artist = None
        self.text_artist = None
        self.hud_artist = None
        self.roi_artist = None
        self.traj_line: Optional[mlines.Line2D] = None

        self.traj_x: List[float] = []
        self.traj_y: List[float] = []

        # Frontier visualization
        self.frontier_artists: List[plt.Circle] = []
        self.best_frontier_artist: Optional[plt.Circle] = None
        self.exit_artist: Optional[plt.Circle] = None

        # Statistics text
        self.stats_text = None

        # Legend
        self._create_legend()

        plt.ion()
        plt.tight_layout()
        plt.show(block=False)

    def _create_legend(self):
        """Create legend for map elements"""
        legend_elements = [
            mlines.Line2D([0], [0], marker='>', color='red', linestyle='None',
                         markersize=10, label='Robot'),
            mlines.Line2D([0], [0], color='blue', linewidth=2, label='Trajectory'),
            mlines.Line2D([0], [0], marker='o', color='yellow', linestyle='None',
                         markersize=8, label='Frontiers'),
            mlines.Line2D([0], [0], marker='o', color='blue', linestyle='None',
                         markersize=10, label='Best Frontier'),
            mlines.Line2D([0], [0], marker='o', color='green', linestyle='None',
                         markersize=12, label='Goal/Exit'),
            mpatches.Patch(facecolor='black', edgecolor='none', label='Obstacles'),
            mpatches.Patch(facecolor='white', edgecolor='none', label='Free Space'),
            mpatches.Patch(facecolor='gray', edgecolor='none', label='Unknown'),
        ]
        self.ax.legend(handles=legend_elements, loc='upper right',
                      fontsize=8, framealpha=0.9)

    def display(self, grid: List[List[int]],
                pose: Optional[Tuple[float, float, float]] = None,
                extra_text: Optional[str] = None,
                roi_rect: Optional[Tuple[int, int, int, int]] = None,
                frontier_points: Optional[List[Tuple[int, int]]] = None,
                best_frontier_point: Optional[Tuple[int, int]] = None,
                spatially_sampled: Optional[List[Tuple[int, int]]] = None,
                exit_pose_pix: Optional[Tuple[int, int]] = None) -> bool:
        """Update display with enhanced visualization"""

        # Update grid image with custom colormap
        if self.img_artist is None:
            self.img_artist = self.ax.imshow(
                grid, cmap='gray', vmin=0, vmax=255,
                origin=self.origin, extent=self.extent_m, zorder=1,
                interpolation='nearest'
            )
        else:
            self.img_artist.set_data(grid)

        # Update robot and trajectory
        if pose is not None:
            x_pix, y_pix, theta_deg = pose
            x_m, y_m = self._pix_to_m(x_pix, y_pix)
            self._draw_robot(x_m, y_m, theta_deg)
            self._update_trajectory(x_m, y_m)

            # Draw frontiers with better visualization
            self._draw_frontiers(spatially_sampled, best_frontier_point)
            self._draw_exit(exit_pose_pix)

        # Update HUD text with better styling
        if extra_text:
            x0 = self.extent_m[0] + 0.05 * (self.extent_m[1] - self.extent_m[0])
            y1 = self.extent_m[3] - 0.05 * (self.extent_m[3] - self.extent_m[2])
            if self.hud_artist is None:
                self.hud_artist = self.ax.text(
                    x0, y1, extra_text, fontsize=9, color="white",
                    va="top", ha="left", zorder=10, family='monospace',
                    bbox=dict(facecolor="black", alpha=0.7, pad=5,
                             edgecolor="cyan", linewidth=1.5, boxstyle='round,pad=0.5')
                )
            else:
                self.hud_artist.set_position((x0, y1))
                self.hud_artist.set_text(extra_text)

        # Update ROI rectangle with better styling
        self._draw_roi(roi_rect, grid)

        # Update statistics
        self._update_stats(pose, len(self.traj_x))

        try:
            self.fig.canvas.draw_idle()
            plt.pause(0.01)
            return True
        except:
            return False

    def _pix_to_m(self, x_pix: float, y_pix: float) -> Tuple[float, float]:
        """Convert pixels to meters"""
        return x_pix * self.meters_per_pixel, y_pix * self.meters_per_pixel

    def _draw_robot(self, x_m: float, y_m: float, theta_deg: float):
        """Draw robot as enhanced arrow with glow effect"""
        if self.car_artist is not None:
            try:
                self.car_artist.remove()
            except:
                pass
            self.car_artist = None

        theta_rad = math.radians(theta_deg)
        dx = self.ROBOT_SHAFT_LEN_M * math.cos(theta_rad)
        dy = self.ROBOT_SHAFT_LEN_M * math.sin(theta_rad)

        # Main robot arrow
        self.car_artist = self.ax.arrow(
            x_m, y_m, dx, dy,
            head_width=self.ROBOT_HEAD_W_M,
            head_length=self.ROBOT_HEAD_LEN_M,
            fc="red", ec="darkred",
            length_includes_head=True,
            zorder=5,
            linewidth=2
        )

        # Update label with better styling
        label = f"Robot: ({x_m:.2f}, {y_m:.2f})m\nθ = {theta_deg:.1f}°"
        if self.text_artist is None:
            self.text_artist = self.ax.text(
                x_m, y_m - 0.15, label, fontsize=9, color="darkred",
                va="top", ha="center", zorder=6, clip_on=True,
                fontweight='bold',
                bbox=dict(facecolor="white", alpha=0.8, pad=3,
                         edgecolor="red", linewidth=1)
            )
        else:
            self.text_artist.set_position((x_m, y_m - 0.15))
            self.text_artist.set_text(label)

    def _update_trajectory(self, x_m: float, y_m: float):
        """Update trajectory line with gradient effect"""
        if not self.show_trajectory:
            return

        self.traj_x.append(x_m)
        self.traj_y.append(y_m)

        if self.traj_line is None:
            self.traj_line = mlines.Line2D(
                self.traj_x, self.traj_y,
                linewidth=2.0, color="blue", zorder=3,
                alpha=0.6, linestyle='-'
            )
            self.ax.add_line(self.traj_line)
        else:
            self.traj_line.set_data(self.traj_x, self.traj_y)

    def _draw_frontiers(self, spatial_sampled: Optional[List[Tuple[int, int]]],
                        best_frontier: Optional[Tuple[int, int]]):
        """Draw frontier points with enhanced styling"""
        # Clear old
        for artist in self.frontier_artists:
            try:
                artist.remove()
            except:
                pass
        self.frontier_artists.clear()

        if self.best_frontier_artist:
            try:
                self.best_frontier_artist.remove()
            except:
                pass
            self.best_frontier_artist = None

        # Draw sampled frontiers (yellow with border)
        if spatial_sampled:
            for x_pix, y_pix in spatial_sampled:
                x_m, y_m = self._pix_to_m(x_pix, y_pix)
                circle = plt.Circle(
                    (x_m, y_m), radius=0.05,
                    color='yellow', alpha=0.7, zorder=4,
                    edgecolor='orange', linewidth=1
                )
                self.ax.add_patch(circle)
                self.frontier_artists.append(circle)

        # Draw best frontier (blue with animation effect)
        if best_frontier:
            x_pix, y_pix = best_frontier
            x_m, y_m = self._pix_to_m(x_pix, y_pix)

            # Outer glow
            glow = plt.Circle(
                (x_m, y_m), radius=0.12,
                color='cyan', alpha=0.3, zorder=4
            )
            self.ax.add_patch(glow)
            self.frontier_artists.append(glow)

            # Main circle
            self.best_frontier_artist = plt.Circle(
                (x_m, y_m), radius=0.08,
                color='blue', alpha=0.9, zorder=5,
                edgecolor='darkblue', linewidth=2
            )
            self.ax.add_patch(self.best_frontier_artist)

    def _draw_exit(self, exit_pix: Optional[Tuple[int, int]]):
        """Draw exit/goal point with pulsing effect"""
        if self.exit_artist:
            try:
                self.exit_artist.remove()
            except:
                pass
            self.exit_artist = None

        if exit_pix:
            x_pix, y_pix = exit_pix
            x_m, y_m = self._pix_to_m(x_pix, y_pix)

            # Outer ring
            outer = plt.Circle(
                (x_m, y_m), radius=0.15,
                color='lightgreen', alpha=0.4, zorder=6,
                linewidth=0, fill=True
            )
            self.ax.add_patch(outer)

            # Main circle
            self.exit_artist = plt.Circle(
                (x_m, y_m), radius=0.10,
                color='green', alpha=0.9, zorder=6,
                linewidth=3, fill=True, edgecolor='darkgreen'
            )
            self.ax.add_patch(self.exit_artist)

            # Center marker
            center = plt.Circle(
                (x_m, y_m), radius=0.03,
                color='white', alpha=1.0, zorder=7
            )
            self.ax.add_patch(center)

    def _draw_roi(self, roi_rect: Optional[Tuple[int, int, int, int]],
                  grid: List[List[int]]):
        """Draw ROI rectangle with enhanced styling"""
        if roi_rect is None:
            if self.roi_artist:
                self.roi_artist.set_visible(False)
            return

        x0_pix, y0_pix, x1_pix, y1_pix = roi_rect
        H, W = len(grid), len(grid[0]) if grid else 0

        xmin, xmax, ymin, ymax = self.extent_m
        sx = (xmax - xmin) / max(1, W)
        sy = (ymax - ymin) / max(1, H)

        if self.origin == 'lower':
            x_m = xmin + x0_pix * sx
            y_m = ymin + y0_pix * sy
            w_m = (x1_pix - x0_pix + 1) * sx
            h_m = (y1_pix - y0_pix + 1) * sy
        else:
            x_m = xmin + x0_pix * sx
            y_m = ymax - (y1_pix + 1) * sy
            w_m = (x1_pix - x0_pix + 1) * sx
            h_m = (y1_pix - y0_pix + 1) * sy

        if self.roi_artist is None:
            self.roi_artist = mpatches.Rectangle(
                (x_m, y_m), w_m, h_m,
                fill=False, linewidth=2, linestyle='--',
                edgecolor='lime', zorder=3, alpha=0.95
            )
            self.ax.add_patch(self.roi_artist)
        else:
            self.roi_artist.set_xy((x_m, y_m))
            self.roi_artist.set_width(w_m)
            self.roi_artist.set_height(h_m)
            self.roi_artist.set_visible(True)

    def _update_stats(self, pose: Optional[Tuple[float, float, float]],
                     traj_length: int):
        """Display statistics in bottom-left corner"""
        if pose is None:
            return

        x_pix, y_pix, theta_deg = pose
        x_m, y_m = self._pix_to_m(x_pix, y_pix)

        stats_text = f"📊 Statistics:\n"
        stats_text += f"Position: ({x_m:.3f}, {y_m:.3f}) m\n"
        stats_text += f"Heading: {theta_deg:.1f}°\n"
        stats_text += f"Path Length: {traj_length} points"

        x0 = self.extent_m[0] + 0.05 * (self.extent_m[1] - self.extent_m[0])
        y0 = self.extent_m[2] + 0.05 * (self.extent_m[3] - self.extent_m[2])

        if self.stats_text is None:
            self.stats_text = self.ax.text(
                x0, y0, stats_text, fontsize=8, color="white",
                va="bottom", ha="left", zorder=10, family='monospace',
                bbox=dict(facecolor="black", alpha=0.7, pad=5,
                         edgecolor="yellow", linewidth=1, boxstyle='round,pad=0.5')
            )
        else:
            self.stats_text.set_position((x0, y0))
            self.stats_text.set_text(stats_text)