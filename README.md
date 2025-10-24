# SLAM Autonomous Navigation Robot System

<div align="center">

![Python](https://img.shields.io/badge/Python-3.13-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

*Real-time Autonomous Exploration and Navigation System Based on BreezySLAM*

[Features](#-features) • [Architecture](#-system-architecture) • [Quick Start](#-quick-start) • [Modules](#-module-details) • [API Documentation](#-api-documentation)

</div>

---

## 📋 Table of Contents

- [Project Overview](#-project-overview)
- [Features](#-features)
- [System Architecture](#-system-architecture)
- [File Structure](#-file-structure)
- [Quick Start](#-quick-start)
- [Module Details](#-module-details)
- [Core Algorithms](#-core-algorithms)
- [Configuration](#-configuration)
- [Data Flow](#-data-flow)
- [Development Guide](#-development-guide)
- [FAQ](#-faq)
- [License](#-license)

---

## 📖 Project Overview

This project is a complete **SLAM Autonomous Navigation Robot System** that implements end-to-end automation from perception, mapping, path planning to motion control. The system uses **Python** as the host computer development language, integrates the **BreezySLAM** algorithm for real-time map construction, and achieves autonomous exploration of unknown environments through **Frontier-based exploration strategies**.

### 🎯 Core Objectives

- **Real-time SLAM Mapping**: LiDAR-based real-time environment mapping and localization
- **Autonomous Exploration**: Automatic exploration of unknown areas without manual intervention
- **Intelligent Navigation**: A* path planning + path smoothing optimization
- **Visualization Monitoring**: Real-time display of maps, paths, and Frontier boundaries
- **Modular Design**: High cohesion and low coupling, easy to extend and maintain

---

## ✨ Features

### 🗺️ SLAM Mapping System
- **RMHC-SLAM Algorithm**: Random Hill-Climbing Monte Carlo SLAM with high-precision pose estimation
- **Encoder Integration**: 10ms high-frequency sampling, complete accumulation of motion increments
- **Incremental Map Updates**: Triggered only during motion state transitions, reducing computational burden
- **Grid Map Representation**: Occupancy probability grid supporting unknown/occupied/free three states

### 🎯 Autonomous Exploration
- **Frontier Detection**: Boundary detection algorithm based on 8-connected region growing
- **Reachability Analysis**: Safe path verification considering obstacle inflation
- **Intelligent Target Selection**: Comprehensive consideration of distance, information gain, and regional distribution
- **Exploration State Machine**: Complete closed-loop of Idle→Planning→Navigating→Executing

### 🛣️ Path Planning
- **A* Optimal Path Search**: Heuristic search guaranteeing shortest path
- **Path Smoothing Optimization**: Catmull-Rom spline smoothing improving motion comfort
- **Dynamic Collision Detection**: Safe distance from obstacles considering robot dimensions
- **Path Tracker**: Segmented tracking supporting turn/straight decomposition

### 📊 Visualization System
- **Real-time Map Rendering**: Occupancy grid, Frontier boundaries, path display
- **Robot Pose Visualization**: Real-time attitude and motion trajectory
- **HUD Information Panel**: System status, encoder statistics, mission progress
- **Debug Information**: Frame drop statistics, SLAM update frequency, queue status

### 🔌 Communication System
- **Dual Frame Format Parsing**: Encoder frame (21 bytes, 10ms) + LiDAR frame (2604 bytes, 200ms)
- **Independent Queue Management**: Encoder queue capacity 100, LiDAR queue capacity 1
- **CRC Verification**: Command frame CRC16-Modbus verification ensuring data integrity
- **Frame Synchronization Mechanism**: cmd_id-based command-response synchronization

---

## 🏗️ System Architecture

### Overall Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    Host Computer System (Python)                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐  │
│  │  Data         │───▶│  SLAM         │───▶│  Mission      │  │
│  │  Acquisition  │    │  Mapping      │    │  Planning     │  │
│  └───────────────┘    └───────────────┘    └───────────────┘  │
│         │                     │                     │          │
│         │                     │                     │          │
│   ┌─────▼─────┐         ┌─────▼─────┐         ┌─────▼─────┐  │
│   │ Serial    │         │ Map       │         │ Path      │  │
│   │ Comm      │         │ Manager   │         │ Planner   │  │
│   │ btlink.py │         │ slam_map/ │         │ planner/  │  │
│   └───────────┘         └───────────┘         └───────────┘  │
│         │                     │                     │          │
│         │                     │                     │          │
│         └─────────────────────┴─────────────────────┘          │
│                              │                                 │
│                    ┌─────────▼──────────┐                     │
│                    │   Visualization    │                     │
│                    │   & Control Layer  │                     │
│                    │  map_visualizer.py │                     │
│                    └────────────────────┘                     │
│                                                                 │
└──────────────────────────┬──────────────────────────────────────┘
                           │ Serial Communication (921600 baud)
                           │ Command / Encoder / LiDAR Frames
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Lower Computer System (STM32F4)                 │
│        [Embedded Firmware - Not covered in this document]       │
└─────────────────────────────────────────────────────────────────┘
```

### Software Module Division

```
SLAM-car/
├─ Core Control Layer
│  ├─ main_robot.py          # Main program entry, system scheduling
│  └─ btlink.py              # Bluetooth/serial communication module
│
├─ Mission Planning Layer (planner/)
│  ├─ mission_controller.py  # Mission controller (exploration state machine)
│  ├─ frontier_detector.py   # Frontier boundary detection
│  ├─ frontier_selector.py   # Frontier target selection strategy
│  ├─ astar_planner.py       # A* path planning algorithm
│  ├─ path_smoother.py       # Path smoothing optimization
│  ├─ command_generator.py   # Motion command generation
│  ├─ reachability.py        # Reachability analysis
│  ├─ region_tracker.py      # Region division and tracking
│  ├─ transform.py           # Coordinate transformation tools
│  └─ planning_config.py     # Planning parameter configuration
│
├─ Map Construction Layer (slam_map/)
│  ├─ occupancy_grid.py      # Occupancy grid map
│  ├─ vote_manager.py        # Voting-based map updates
│  ├─ map_visualizer.py      # Map visualization
│  └─ draw_utils.py          # Drawing utility functions
│
├─ Communication Layer (serial_io/)
│  └─ __init__.py            # Serial IO tools
│
└─ Configuration & Testing
   ├─ bt_test.py             # Bluetooth communication test
   ├─ requirement.txt        # Python dependency list
   └─ README.md              # This document
```


---

## 📁 File Structure

### Detailed Directory Tree

```
SLAM-car/
│
├─ Main Program Files
│  ├─ main_robot.py                 # Main program: system initialization, multi-thread scheduling
│  ├─ btlink.py                     # Bluetooth communication: frame parsing, queue management, data I/O
│  ├─ bt_test.py                    # Communication test: for testing serial transmission/reception
│  └─ requirement.txt               # Dependency list: numpy, opencv, pyserial, etc.
│
├─ planner/                         # Mission Planning Module
│  ├─ mission_controller.py         # Exploration mission controller
│  │   ├─ MissionController class   # State machine: IDLE/PLANNING/NAVIGATING
│  │   ├─ run()                     # Main loop: Frontier detection→path planning→command generation
│  │   └─ check_goal_reached()      # Goal arrival judgment
│  │
│  ├─ frontier_detector.py          # Frontier boundary detector
│  │   ├─ find_frontiers()          # 8-connected region growing algorithm
│  │   ├─ filter_small_regions()    # Filter small regions
│  │   └─ compute_centroids()       # Compute region centroids
│  │
│  ├─ frontier_selector.py          # Frontier target selection strategy
│  │   ├─ select_best_frontier()    # Comprehensive scoring: distance + information gain
│  │   ├─ compute_scores()          # Calculate score for each Frontier
│  │   └─ filter_visited()          # Filter visited regions
│  │
│  ├─ astar_planner.py              # A* path planner
│  │   ├─ plan_path()               # A* search main function
│  │   ├─ heuristic()               # Euclidean distance heuristic function
│  │   └─ reconstruct_path()        # Path backtracking
│  │
│  ├─ path_smoother.py              # Path smoothing optimizer
│  │   ├─ smooth_path()             # Catmull-Rom spline interpolation
│  │   ├─ interpolate_spline()      # Spline curve calculation
│  │   └─ resample_path()           # Path resampling
│  │
│  ├─ command_generator.py          # Motion command generator
│  │   ├─ generate_commands()       # Path→command sequence conversion
│  │   ├─ decompose_motion()        # Decompose into turn+straight
│  │   └─ PathTracker class         # Path tracker
│  │
│  ├─ reachability.py               # Reachability analysis tool
│  │   ├─ is_reachable()            # Check if target point is reachable
│  │   └─ check_path_collision()    # Path collision detection
│  │
│  ├─ region_tracker.py             # Region tracker
│  │   ├─ mark_visited()            # Mark visited regions
│  │   └─ get_unvisited_regions()   # Get unvisited regions
│  │
│  ├─ transform.py                  # Coordinate transformation tools
│  │   ├─ pixel_to_world()          # Pixel coordinates→world coordinates
│  │   ├─ world_to_pixel()          # World coordinates→pixel coordinates
│  │   └─ rotate_point()            # Coordinate rotation
│  │
│  └─ planning_config.py            # Planning parameter configuration
│      ├─ FRONTIER_MIN_SIZE         # Minimum Frontier size
│      ├─ PATH_SMOOTHNESS           # Path smoothness
│      └─ ROBOT_RADIUS              # Robot radius
│
├─ slam_map/                        # SLAM Map Module
│  ├─ occupancy_grid.py             # Occupancy grid map
│  │   ├─ OccupancyGrid class       # Map data structure
│  │   ├─ update_cell()             # Cell update
│  │   ├─ get_occupancy()           # Query occupancy status
│  │   └─ get_free_cells()          # Get free cells
│  │
│  ├─ vote_manager.py               # Voting-based map updates
│  │   ├─ VoteManager class         # Manage voting weights
│  │   ├─ cast_vote()               # Voting update
│  │   └─ update_probabilities()    # Probability update
│  │
│  ├─ map_visualizer.py             # Map visualization
│  │   ├─ MapVisualizer class       # OpenCV visualization
│  │   ├─ draw_map()                # Draw grid map
│  │   ├─ draw_robot()              # Draw robot pose
│  │   ├─ draw_frontiers()          # Draw Frontier boundaries
│  │   ├─ draw_path()               # Draw path
│  │   └─ draw_hud()                # Draw HUD information
│  │
│  └─ draw_utils.py                 # Drawing utility functions
│      ├─ draw_arrow()              # Draw arrow
│      ├─ draw_circle()             # Draw circle
│      └─ draw_text()               # Draw text
│
├─ serial_io/                       # Serial Communication Module
│  └─ __init__.py                   # Serial tool initialization
│
└─ stm32/                           # Embedded Firmware (Not covered in this document)
   └─ [Lower computer code structure see next section]
```

### Embedded Firmware Structure

```
stm32/                              # STM32F4 Embedded Firmware
│
├─ Core/                            # Core Code
│  ├─ Inc/                          # Header Files
│  │  ├─ main.h                     # Main program header
│  │  ├─ communication.h            # Communication protocol
│  │  ├─ encoder.h                  # Encoder interface
│  │  ├─ motor.h                    # Motor control
│  │  ├─ ax_laser.h                 # LiDAR interface
│  │  └─ ...                        # Other peripheral headers
│  │
│  └─ Src/                          # Source Files
│     ├─ main.c                     # Main program: initialization, state machine
│     ├─ communication.c            # Communication: frame transmission, command reception
│     ├─ encoder.c                  # Encoder: cumulative value reading
│     ├─ motor.c                    # Motor: PWM control
│     ├─ ax_laser_task.c            # LiDAR task: data acquisition
│     └─ ...                        # Other source files
│
├─ Drivers/                         # HAL Driver Library
│  ├─ CMSIS/                        # CMSIS Standard Library
│  └─ STM32F4xx_HAL_Driver/         # STM32 HAL Library
│
└─ MDK-ARM/                         # Keil MDK Project Files
   ├─ test-pwm.uvprojx              # Keil project file
   └─ startup_stm32f446xx.s         # Startup file
```

---

## 🚀 Quick Start

### Environment Requirements

- **Python**: 3.8+ (3.13 recommended)
- **Operating System**: Windows / Linux / macOS
- **Serial Port**: Bluetooth module or USB-to-Serial converter

### Install Dependencies

```bash
# Clone repository
git clone https://github.com/yourusername/SLAM-car.git
cd SLAM-car

# Install Python dependencies
pip install -r requirement.txt
```

**Dependency List** (`requirement.txt`):
```
numpy>=1.20.0
opencv-python>=4.5.0
pyserial>=3.5
breezyslam>=0.2.0
```

### Run Program

#### 1. Basic Run (Automatic Exploration Mode)

```bash
python main_robot.py --port COM7 --baud 921600
```

#### 2. Run with Parameters

```bash
python main_robot.py \
    --port COM7 \
    --baud 921600 \
    --wheel-radius 0.0325 \
    --half-wheelbase 0.084 \
    --cpr 1650 \
    --map-size 8000 \
    --map-quality 10
```

#### 3. Test Mode

```bash
# Test Bluetooth communication
python bt_test.py

# View help
python main_robot.py --help
```

### Command Line Arguments

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `--port` | str | COM7 | Serial port number |
| `--baud` | int | 921600 | Baud rate |
| `--wheel-radius` | float | 0.0325 | Wheel radius (meters) |
| `--half-wheelbase` | float | 0.084 | Half wheelbase (meters) |
| `--cpr` | int | 1650 | Encoder counts per revolution |
| `--map-size` | int | 8000 | Map size (mm) |
| `--map-quality` | int | 10 | Map resolution (mm/pixel) |


---

## 🔧 Module Details

### 1️⃣ Communication Module (`btlink.py`)

#### Function Overview

Responsible for serial communication with the lower computer, including data frame parsing, command sending, and data reception.

#### Core Classes

**`BluetoothLink`**
```python
class BluetoothLink:
    def __init__(self, port: str, baudrate: int = 921600):
        """Initialize Bluetooth connection"""
        
    def send_command(self, cmd_id: int, turn_rad: float, distance_m: float):
        """Send motion command (with CRC verification)"""
        
    def get_encoder_frame(self) -> Optional[EncoderFrame]:
        """Get encoder data frame (non-blocking)"""
        
    def get_lidar_frame(self) -> Optional[LidarFrame]:
        """Get LiDAR data frame (non-blocking)"""
        
    def close(self):
        """Close connection and cleanup resources"""
```

#### Data Frame Formats

**Encoder Frame** (21 bytes, 10ms period)
```
[0xFD 0xDF] + time_us(4) + cmd_id(2) + status(1) + 
encoder_l(4) + encoder_r(4) + yaw(4)
```

**LiDAR Frame** (2604 bytes, 200ms period)
```
[0x55 0xAA] + data_count(2) + [quality(1) + angle_q6(2) + dist_q2(2)] × 520
```

**Command Frame** (16 bytes, on-demand)
```
[0xAA 0x55] + len(2) + cmd_id(2) + turn_rad(4) + distance_m(4) + CRC16(2)
```

#### Queue Management Strategy

- **Encoder Queue**: `maxsize=100` (high-frequency data, cache more frames)
- **LiDAR Queue**: `maxsize=1` (low-frequency data, keep only latest frame)
- **Auto Discard**: When queue is full, LiDAR data automatically overwrites old data

#### Usage Example

```python
# Create connection
bt = BluetoothLink(port="COM7", baudrate=921600)

# Send motion command
bt.send_command(cmd_id=1, turn_rad=0.5, distance_m=0.3)

# Get data
encoder_frame = bt.get_encoder_frame()
lidar_frame = bt.get_lidar_frame()

# Close connection
bt.close()
```

---

### 2️⃣ SLAM Mapping Module (`slam_map/`)

#### Function Overview

Implements real-time map construction based on BreezySLAM, including occupancy grid management, voting-based updates, and visualization rendering.

#### Core Classes

**`OccupancyGrid`** - Occupancy Grid Map
```python
class OccupancyGrid:
    def __init__(self, size_mm: int, quality_mm: int):
        """
        Initialize grid map
        
        Args:
            size_mm: Map size (millimeters)
            quality_mm: Resolution (millimeters/pixel)
        """
        
    def update_cell(self, x: int, y: int, occupied: bool):
        """Update occupancy status of a single grid cell"""
        
    def get_occupancy(self, x: int, y: int) -> int:
        """
        Query grid occupancy status
        
        Returns:
            0: Unknown, 127: Free, 255: Occupied
        """
        
    def get_free_cells(self) -> List[Tuple[int, int]]:
        """Get all free cell coordinates"""
```

**`MapVisualizer`** - Map Visualization
```python
class MapVisualizer:
    def draw_map(self, map_data: np.ndarray):
        """Draw grid map (grayscale image)"""
        
    def draw_robot(self, x: float, y: float, theta: float):
        """Draw robot pose (red triangle)"""
        
    def draw_frontiers(self, frontiers: List[List[Tuple[int, int]]]):
        """Draw Frontier boundaries (green contours)"""
        
    def draw_path(self, path: List[Tuple[float, float]]):
        """Draw planned path (blue line)"""
        
    def draw_hud(self, info: dict):
        """Draw HUD information panel"""
        
    def show(self):
        """Show window (with keyboard interaction)"""
```

#### SLAM Update Flow

```
Encoder Accumulation Thread (100Hz)
    ↓
Accumulate motion increments (Δleft, Δright)
    ↓
Detect state transition (1→2 or 2→0)
    ↓
Trigger SLAM update
    ↓
slam.update(laser scan, accumulated increments)
    ↓
Update robot pose + map
    ↓
Reset accumulator
```

#### Usage Example

```python
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from slam_map.occupancy_grid import OccupancyGrid
from slam_map.map_visualizer import MapVisualizer

# Initialize SLAM
laser = Laser(520, 5.0, 360.0, 8000, 0, 0)
slam = RMHC_SLAM(laser, 8000, 10)
map_data = bytearray(800 * 800)

# Initialize visualization
viz = MapVisualizer(width=800, height=800)

# SLAM update
slam.update(lidar_scan, left_mm, right_mm)
x, y, theta = slam.getpos()

# Draw map
slam.getmap(map_data)
viz.draw_map(np.frombuffer(map_data, dtype=np.uint8).reshape(800, 800))
viz.draw_robot(x, y, theta)
viz.show()
```


---

## ❓ FAQ

### Q1: Unable to connect to serial port after program starts

**Solution**:
1. Check if serial port number is correct: `python -m serial.tools.list_ports`
2. Confirm baud rate matches: 921600
3. Check if serial port is occupied by another program
4. Windows users: Check COM port in Device Manager

### Q2: SLAM map not updating

**Possible Causes**:
1. Encoder data not received → Check encoder queue statistics
2. Poor LiDAR data quality → View LiDAR point count in HUD
3. State transition not triggered → Confirm status value changes normally
4. Motion increment too small → Check encoder accumulated values

**Debugging Method**:
```python
# Add debug logging
print(f"Encoder frames: {encoder_count}")
print(f"Status: {encoder_frame.status}")
print(f"Accumulated: left={acc_left}, right={acc_right}")
```

### Q3: Path planning fails

**Troubleshooting Steps**:
1. Check if Frontier detected: `len(frontiers) > 0`
2. Confirm target point reachability: `reachability.is_reachable(goal)`
3. View A* search logs: Check for infinite loops
4. Increase obstacle inflation radius: `OBSTACLE_INFLATION += 1`

### Q4: Robot keeps spinning

**Cause Analysis**:
- cmd_id synchronization failure
- State machine stuck in a state
- Command queue not cleared

**Solution**:
```python
# Reset state machine
controller.reset()

# Clear command queue
while not command_queue.empty():
    command_queue.get()

# Force idle state
robot_status = 0
```

### Q5: Encoder accumulated value anomaly

**Checklist**:
1. Is lower computer sending accumulated value instead of increment
2. Is accumulator reset after SLAM trigger
3. Is encoder CPR correctly configured

**Correct accumulation logic**:
```python
# Correct
if frame_count > 0:
    delta_left = current_left - last_left
    acc_left += delta_left

# Wrong (lower computer sends increment)
acc_left += current_left  # ❌ Don't do this
```

### Q6: "Ghost" artifacts appear on map

**Phenomenon**: Non-existent obstacles appear on map

**Cause**: SLAM pose estimation drift

**Solution**:
1. Increase SLAM update frequency
2. Improve encoder data quality
3. Add loop closure detection
4. Use higher precision odometry model

### Q7: System CPU usage too high

**Optimization Suggestions**:
1. Reduce map visualization frequency: `cv2.waitKey(100)` → `cv2.waitKey(200)`
2. Reduce SLAM update frequency: Trigger only when necessary
3. Optimize A* search: Use JPS algorithm
4. Multi-threading optimization: Move visualization to separate thread

---

## 📜 License

This project is licensed under the **MIT License**.

```
MIT License

Copyright (c) 2025 SLAM-car Project

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 🙏 Acknowledgements

This project uses the following open-source libraries:

- **BreezySLAM**: SLAM algorithm core - https://github.com/simondlevy/BreezySLAM
- **NumPy**: Scientific computing foundation - https://numpy.org/
- **OpenCV**: Computer vision and visualization - https://opencv.org/
- **PySerial**: Serial communication - https://github.com/pyserial/pyserial

Special thanks to all developers who contribute to the open-source community!

---

## 📧 Contact

- **Project Homepage**: https://github.com/yourusername/SLAM-car
- **Issue Tracker**: https://github.com/yourusername/SLAM-car/issues
- **Email**: your.email@example.com

---

<div align="center">

**⭐ If this project helps you, please give us a Star! ⭐**

Made with ❤️ by SLAM-car Team

</div>
───┐
               │ Frontier     │   │  Path     │   │  Command    │
               │   Detection  │──▶│  Planning │──▶│  Generation │
               └──────────────┘   └───────────┘   └──────┬──────┘
                                                          │
                                                          │
┌─────────────────────────────────────────────────────────────────┐
│                       Command Execution Layer                    │
└─────────────────────────────────────────────────────────────────┘
                                                          │
                                                   ┌──────▼──────┐
                                                   │ Serial Send │
                                                   │ cmd_id Sync │
                                                   └──────┬──────┘
                                                          │
                                                          ▼
                                                 [Lower Computer Exec]
```

### Timing Diagram

```
Timeline ────────────────────────────────────────────────────────────▶

Encoder    ┃     ┃     ┃     ┃     ┃     ┃     ┃     ┃     ┃  
10ms int.  0ms  10ms  20ms  30ms  40ms  50ms  60ms  70ms  80ms ...

LiDAR      ┃                                   ┃
200ms int. 0ms                                200ms                400ms

SLAM           △ (not triggered)                △ (triggered!)
Update          status=1(turning)                 state trans 1→2

Mission        ◇                              ◇
Planning        Frontier detection              Path planning

Command       ●                               ●
Send           cmd_id=1                        cmd_id=2
              (turn 30°)                      (straight 0.5m)
```

---

## 🛠️ Development Guide

### Adding a New Planner

```python
# planner/my_planner.py

class MyCustomPlanner:
    """Custom path planner"""
    
    def __init__(self, config):
        self.config = config
    
    def plan_path(self, start, goal, map_data):
        """
        Implement your planning algorithm
        
        Args:
            start: Start point (x, y)
            goal: Goal point (x, y)
            map_data: Map data (numpy array)
            
        Returns:
            path: List of path points [(x1, y1), (x2, y2), ...]
        """
        # Your algorithm implementation
        pass

# Usage in main_robot.py
from planner.my_planner import MyCustomPlanner

planner = MyCustomPlanner(config)
path = planner.plan_path(start, goal, map_data)
```

### Adding a New Frontier Selection Strategy

```python
# planner/my_selector.py

class MyFrontierSelector:
    """Custom Frontier selection strategy"""
    
    def select_best_frontier(self, frontiers, robot_pos):
        """
        Select best Frontier based on custom strategy
        
        Args:
            frontiers: List of Frontiers
            robot_pos: Current robot position
            
        Returns:
            best_frontier: Best Frontier centroid
        """
        scores = []
        for frontier in frontiers:
            score = self.compute_score(frontier, robot_pos)
            scores.append(score)
        
        best_idx = np.argmax(scores)
        return self.compute_centroid(frontiers[best_idx])
    
    def compute_score(self, frontier, robot_pos):
        """
        Calculate Frontier score
        
        Example: Comprehensive consideration of distance, size, direction
        """
        centroid = self.compute_centroid(frontier)
        
        # Distance factor (closer = higher score)
        distance = euclidean_distance(robot_pos, centroid)
        distance_score = 1.0 / (1.0 + distance / 1000.0)
        
        # Size factor (larger area = higher score)
        size_score = len(frontier) / 100.0
        
        # Direction factor (aligned with current heading = higher score)
        direction_score = compute_direction_alignment(robot_pos, centroid)
        
        # Weighted combination
        total_score = (0.5 * distance_score + 
                      0.3 * size_score + 
                      0.2 * direction_score)
        
        return total_score
```

### Extending Visualization Features

```python
# slam_map/custom_viz.py

from slam_map.map_visualizer import MapVisualizer

class EnhancedVisualizer(MapVisualizer):
    """Enhanced visualizer"""
    
    def draw_heatmap(self, data):
        """Draw heatmap"""
        heatmap = cv2.applyColorMap(data, cv2.COLORMAP_JET)
        self.image = cv2.addWeighted(self.image, 0.7, heatmap, 0.3, 0)
    
    def draw_trajectory(self, trajectory):
        """Draw historical trajectory"""
        points = np.array(trajectory, dtype=np.int32)
        cv2.polylines(self.image, [points], False, (255, 255, 0), 2)
    
    def draw_debug_info(self, debug_data):
        """Draw debug information"""
        y_offset = 400
        for key, value in debug_data.items():
            text = f"{key}: {value}"
            cv2.putText(self.image, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y_offset += 20
```

### Unit Testing

```python
# tests/test_astar.py

import unittest
from planner.astar_planner import AStarPlanner
import numpy as np

class TestAStarPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = AStarPlanner()
        self.map_data = np.zeros((100, 100), dtype=np.uint8)
    
    def test_straight_path(self):
        """Test straight path"""
        start = (10, 10)
        goal = (10, 90)
        path = self.planner.plan_path(start, goal, self.map_data)
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
    
    def test_obstacle_avoidance(self):
        """Test obstacle avoidance"""
        # Add obstacle wall
        self.map_data[50, 10:90] = 255
        
        start = (10, 50)
        goal = (90, 50)
        path = self.planner.plan_path(start, goal, self.map_data)
        
        self.assertIsNotNone(path)
        # Verify path avoids obstacles
        for (x, y) in path:
            self.assertNotEqual(self.map_data[x, y], 255)
    
    def test_no_path(self):
        """Test no path scenario"""
        # Completely surround goal with obstacles
        self.map_data[80:90, 80:90] = 255
        
        start = (10, 10)
        goal = (85, 85)
        path = self.planner.plan_path(start, goal, self.map_data)
        
        self.assertEqual(len(path), 0)

if __name__ == '__main__':
    unittest.main()
```

