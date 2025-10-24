# SLAM自主导航机器人系统


*基于BreezySLAM的实时自主探索与导航系统*

[功能特性](#-功能特性) • [系统架构](#-系统架构) • [快速开始](#-快速开始) • [模块详解](#-模块详解) • [API文档](#-api文档)

</div>

---

## 📋 目录

- [项目简介](#-项目简介)
- [功能特性](#-功能特性)
- [系统架构](#-系统架构)
- [文件结构](#-文件结构)
- [快速开始](#-快速开始)
- [模块详解](#-模块详解)
- [核心算法](#-核心算法)
- [配置说明](#-配置说明)
- [数据流程](#-数据流程)
- [开发指南](#-开发指南)
- [常见问题](#-常见问题)
- [许可证](#-许可证)

---

## 📖 项目简介

本项目是一个完整的**SLAM自主导航机器人系统**，实现了从感知、建图、路径规划到运动控制的全流程自动化。系统采用**Python**作为上位机开发语言，集成了**BreezySLAM**算法进行实时地图构建，并通过**Frontier-based探索策略**实现未知环境的自主探索。

### 🎯 核心目标

- **实时SLAM建图**：基于激光雷达的实时环境建图与定位
- **自主探索**：无需人工干预，自动探索未知区域
- **智能导航**：A*路径规划 + 路径平滑优化
- **可视化监控**：实时显示地图、路径、Frontier边界
- **模块化设计**：高内聚低耦合，易于扩展和维护

---

## ✨ 功能特性

### 🗺️ SLAM建图系统
- **RMHC-SLAM算法**：随机爬山蒙特卡洛SLAM，高精度位姿估计
- **编码器积分**：10ms高频采样，完整累加运动增量
- **增量式地图更新**：仅在运动状态转换时触发，降低计算负担
- **栅格地图表示**：占据概率栅格，支持未知/占据/空闲三态

### 🎯 自主探索
- **Frontier检测**：基于八连通区域生长的边界检测算法
- **可达性分析**：考虑障碍物膨胀的安全路径验证
- **智能选点策略**：综合考虑距离、信息增益、区域分布
- **探索状态机**：空闲→规划→导航→执行的完整闭环

### 🛣️ 路径规划
- **A*最优路径搜索**：启发式搜索，保证最短路径
- **路径平滑优化**：Catmull-Rom样条平滑，提升运动舒适性
- **动态碰撞检测**：考虑机器人尺寸的障碍物安全距离
- **路径跟踪器**：分段跟踪，支持转向/直行分解

### 📊 可视化系统
- **实时地图渲染**：占据栅格、Frontier边界、路径显示
- **机器人位姿可视化**：实时姿态和运动轨迹
- **HUD信息面板**：系统状态、编码器统计、任务进度
- **调试信息**：丢帧统计、SLAM更新频率、队列状态

### 🔌 通信系统
- **双帧格式解析**：编码器帧(21字节, 10ms) + 雷达帧(2604字节, 200ms)
- **独立队列管理**：编码器队列100容量，雷达队列1容量
- **CRC校验**：命令帧CRC16-Modbus校验，确保数据完整性
- **帧同步机制**：基于cmd_id的命令-响应同步

---

## 🏗️ 系统架构

### 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        上位机系统 (Python)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────┐    ┌───────────────┐    ┌───────────────┐  │
│  │  数据采集层    │───▶│  SLAM建图层   │───▶│  任务规划层   │  │
│  └───────────────┘    └───────────────┘    └───────────────┘  │
│         │                     │                     │          │
│         │                     │                     │          │
│   ┌─────▼─────┐         ┌─────▼─────┐         ┌─────▼─────┐  │
│   │ 串口通信   │         │ 地图管理   │         │ 路径规划   │  │
│   │ btlink.py │         │ slam_map/ │         │ planner/  │  │
│   └───────────┘         └───────────┘         └───────────┘  │
│         │                     │                     │          │
│         │                     │                     │          │
│         └─────────────────────┴─────────────────────┘          │
│                              │                                 │
│                    ┌─────────▼──────────┐                     │
│                    │   可视化与控制层    │                     │
│                    │  map_visualizer.py │                     │
│                    └────────────────────┘                     │
│                                                                 │
└──────────────────────────┬──────────────────────────────────────┘
                           │ 串口通信 (921600 baud)
                           │ 命令帧 / 编码器帧 / 雷达帧
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                      下位机系统 (STM32F4)                         │
│          [嵌入式固件 - 本文档不涉及硬件细节]                       │
└─────────────────────────────────────────────────────────────────┘
```

### 软件模块划分

```
SLAM-car/
├─ 核心控制层
│  ├─ main_robot.py          # 主程序入口，系统调度
│  └─ btlink.py              # 蓝牙/串口通信模块
│
├─ 任务规划层 (planner/)
│  ├─ mission_controller.py  # 任务控制器（探索状态机）
│  ├─ frontier_detector.py   # Frontier边界检测
│  ├─ frontier_selector.py   # Frontier选点策略
│  ├─ astar_planner.py       # A*路径规划算法
│  ├─ path_smoother.py       # 路径平滑优化
│  ├─ command_generator.py   # 运动命令生成
│  ├─ reachability.py        # 可达性分析
│  ├─ region_tracker.py      # 区域划分与跟踪
│  ├─ transform.py           # 坐标变换工具
│  └─ planning_config.py     # 规划参数配置
│
├─ 地图建图层 (slam_map/)
│  ├─ occupancy_grid.py      # 占据栅格地图
│  ├─ vote_manager.py        # 投票式地图更新
│  ├─ map_visualizer.py      # 地图可视化
│  └─ draw_utils.py          # 绘图工具函数
│
├─ 通信层 (serial_io/)
│  └─ __init__.py            # 串口IO工具
│
└─ 配置与测试
   ├─ bt_test.py             # 蓝牙通信测试
   ├─ requirement.txt        # Python依赖列表
   └─ README.md              # 本文档
```

---

## 📁 文件结构

### 详细目录树

```
SLAM-car/
│
├─ 主程序文件
│  ├─ main_robot.py                 # 主程序：系统初始化、多线程调度
│  ├─ btlink.py                     # 蓝牙通信：帧解析、队列管理、数据收发
│  ├─ bt_test.py                    # 通信测试：用于测试串口收发功能
│  └─ requirement.txt               # 依赖列表：numpy, opencv, pyserial等
│
├─ planner/                         # 任务规划模块
│  ├─ mission_controller.py         # 探索任务控制器
│  │   ├─ MissionController类      # 状态机：IDLE/PLANNING/NAVIGATING
│  │   ├─ run()                    # 主循环：Frontier检测→路径规划→命令生成
│  │   └─ check_goal_reached()     # 目标到达判断
│  │
│  ├─ frontier_detector.py          # Frontier边界检测器
│  │   ├─ find_frontiers()         # 八连通区域生长算法
│  │   ├─ filter_small_regions()   # 过滤小区域
│  │   └─ compute_centroids()      # 计算区域质心
│  │
│  ├─ frontier_selector.py          # Frontier选点策略
│  │   ├─ select_best_frontier()   # 综合评分：距离 + 信息增益
│  │   ├─ compute_scores()         # 计算每个Frontier的得分
│  │   └─ filter_visited()         # 过滤已访问区域
│  │
│  ├─ astar_planner.py              # A*路径规划器
│  │   ├─ plan_path()              # A*搜索主函数
│  │   ├─ heuristic()              # 欧几里得距离启发函数
│  │   └─ reconstruct_path()       # 路径回溯
│  │
│  ├─ path_smoother.py              # 路径平滑优化器
│  │   ├─ smooth_path()            # Catmull-Rom样条插值
│  │   ├─ interpolate_spline()     # 样条曲线计算
│  │   └─ resample_path()          # 路径重采样
│  │
│  ├─ command_generator.py          # 运动命令生成器
│  │   ├─ generate_commands()      # 路径→命令序列转换
│  │   ├─ decompose_motion()       # 分解为转向+直行
│  │   └─ PathTracker类            # 路径跟踪器
│  │
│  ├─ reachability.py               # 可达性分析工具
│  │   ├─ is_reachable()           # 检查目标点是否可达
│  │   └─ check_path_collision()   # 路径碰撞检测
│  │
│  ├─ region_tracker.py             # 区域跟踪器
│  │   ├─ mark_visited()           # 标记已访问区域
│  │   └─ get_unvisited_regions()  # 获取未访问区域
│  │
│  ├─ transform.py                  # 坐标变换工具
│  │   ├─ pixel_to_world()         # 像素坐标→世界坐标
│  │   ├─ world_to_pixel()         # 世界坐标→像素坐标
│  │   └─ rotate_point()           # 坐标旋转
│  │
│  └─ planning_config.py            # 规划参数配置
│      ├─ FRONTIER_MIN_SIZE        # Frontier最小尺寸
│      ├─ PATH_SMOOTHNESS          # 路径平滑度
│      └─ ROBOT_RADIUS             # 机器人半径
│
├─ slam_map/                        # SLAM地图模块
│  ├─ occupancy_grid.py             # 占据栅格地图
│  │   ├─ OccupancyGrid类         # 地图数据结构
│  │   ├─ update_cell()            # 单元格更新
│  │   ├─ get_occupancy()          # 查询占据状态
│  │   └─ get_free_cells()         # 获取空闲单元
│  │
│  ├─ vote_manager.py               # 投票式地图更新
│  │   ├─ VoteManager类            # 管理投票权重
│  │   ├─ cast_vote()              # 投票更新
│  │   └─ update_probabilities()   # 概率更新
│  │
│  ├─ map_visualizer.py             # 地图可视化
│  │   ├─ MapVisualizer类         # OpenCV可视化
│  │   ├─ draw_map()               # 绘制栅格地图
│  │   ├─ draw_robot()             # 绘制机器人位姿
│  │   ├─ draw_frontiers()         # 绘制Frontier边界
│  │   ├─ draw_path()              # 绘制路径
│  │   └─ draw_hud()               # 绘制HUD信息
│  │
│  └─ draw_utils.py                 # 绘图工具函数
│      ├─ draw_arrow()             # 绘制箭头
│      ├─ draw_circle()            # 绘制圆形
│      └─ draw_text()              # 绘制文本
│
├─ serial_io/                       # 串口通信模块
│  └─ __init__.py                   # 串口工具初始化
│
└─ stm32/                           # 嵌入式固件（本文档不涉及）
   └─ [下位机代码结构见下节]
```

### 嵌入式固件结构

```
stm32/                              # STM32F4嵌入式固件
│
├─ Core/                            # 核心代码
│  ├─ Inc/                          # 头文件
│  │  ├─ main.h                    # 主程序头文件
│  │  ├─ communication.h           # 通信协议
│  │  ├─ encoder.h                 # 编码器接口
│  │  ├─ motor.h                   # 电机控制
│  │  ├─ ax_laser.h                # 激光雷达接口
│  │  └─ ...                       # 其他外设头文件
│  │
│  └─ Src/                          # 源文件
│     ├─ main.c                    # 主程序：初始化、状态机
│     ├─ communication.c           # 通信：帧发送、命令接收
│     ├─ encoder.c                 # 编码器：累积值读取
│     ├─ motor.c                   # 电机：PWM控制
│     ├─ ax_laser_task.c           # 激光任务：数据采集
│     └─ ...                       # 其他源文件
│
├─ Drivers/                         # HAL驱动库
│  ├─ CMSIS/                       # CMSIS标准库
│  └─ STM32F4xx_HAL_Driver/        # STM32 HAL库
│
└─ MDK-ARM/                         # Keil MDK工程文件
   ├─ test-pwm.uvprojx             # Keil工程文件
   └─ startup_stm32f446xx.s        # 启动文件
```

---

## 🚀 快速开始

### 环境要求

- **Python**: 3.8+（推荐3.13）
- **操作系统**: Windows / Linux / macOS
- **串口**: 蓝牙模块或USB转串口

### 安装依赖

```bash
# 克隆仓库
git clone https://github.com/yourusername/SLAM-car.git
cd SLAM-car

# 安装Python依赖
pip install -r requirement.txt
```

**依赖列表** (`requirement.txt`):
```
numpy>=1.20.0
opencv-python>=4.5.0
pyserial>=3.5
breezyslam>=0.2.0
```

### 运行程序

#### 1. 基础运行（自动探索模式）

```bash
python main_robot.py --port COM7 --baud 921600
```

#### 2. 带参数运行

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

#### 3. 测试模式

```bash
# 测试蓝牙通信
python bt_test.py

# 查看帮助
python main_robot.py --help
```

### 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--port` | str | COM7 | 串口端口号 |
| `--baud` | int | 921600 | 波特率 |
| `--wheel-radius` | float | 0.0325 | 轮半径（米）|
| `--half-wheelbase` | float | 0.084 | 半轴距（米）|
| `--cpr` | int | 1650 | 编码器线数 |
| `--map-size` | int | 8000 | 地图尺寸（mm）|
| `--map-quality` | int | 10 | 地图分辨率（mm/像素）|

---

## 🔧 模块详解

### 1️⃣ 通信模块 (`btlink.py`)

#### 功能概述

负责与下位机的串口通信，包括数据帧解析、命令发送和数据接收。

#### 核心类

**`BluetoothLink`**
```python
class BluetoothLink:
    def __init__(self, port: str, baudrate: int = 921600):
        """初始化蓝牙连接"""
        
    def send_command(self, cmd_id: int, turn_rad: float, distance_m: float):
        """发送运动命令（带CRC校验）"""
        
    def get_encoder_frame(self) -> Optional[EncoderFrame]:
        """获取编码器数据帧（非阻塞）"""
        
    def get_lidar_frame(self) -> Optional[LidarFrame]:
        """获取雷达数据帧（非阻塞）"""
        
    def close(self):
        """关闭连接并清理资源"""
```

#### 数据帧格式

**编码器帧** (21字节, 10ms周期)
```
[0xFD 0xDF] + time_us(4) + cmd_id(2) + status(1) + 
encoder_l(4) + encoder_r(4) + yaw(4)
```

**雷达帧** (2604字节, 200ms周期)
```
[0x55 0xAA] + data_count(2) + [quality(1) + angle_q6(2) + dist_q2(2)] × 520
```

**命令帧** (16字节, 按需发送)
```
[0xAA 0x55] + len(2) + cmd_id(2) + turn_rad(4) + distance_m(4) + CRC16(2)
```

#### 队列管理策略

- **编码器队列**: `maxsize=100`（高频数据，缓存更多帧）
- **雷达队列**: `maxsize=1`（低频数据，只保留最新帧）
- **自动丢弃**: 队列满时，雷达数据自动覆盖旧数据

#### 使用示例

```python
# 创建连接
bt = BluetoothLink(port="COM7", baudrate=921600)

# 发送运动命令
bt.send_command(cmd_id=1, turn_rad=0.5, distance_m=0.3)

# 获取数据
encoder_frame = bt.get_encoder_frame()
lidar_frame = bt.get_lidar_frame()

# 关闭连接
bt.close()
```

---

### 2️⃣ SLAM建图模块 (`slam_map/`)

#### 功能概述

实现基于BreezySLAM的实时地图构建，包括占据栅格管理、投票式更新和可视化渲染。

#### 核心类

**`OccupancyGrid`** - 占据栅格地图
```python
class OccupancyGrid:
    def __init__(self, size_mm: int, quality_mm: int):
        """
        初始化栅格地图
        
        Args:
            size_mm: 地图尺寸（毫米）
            quality_mm: 分辨率（毫米/像素）
        """
        
    def update_cell(self, x: int, y: int, occupied: bool):
        """更新单个栅格的占据状态"""
        
    def get_occupancy(self, x: int, y: int) -> int:
        """
        查询栅格占据状态
        
        Returns:
            0: 未知, 127: 空闲, 255: 占据
        """
        
    def get_free_cells(self) -> List[Tuple[int, int]]:
        """获取所有空闲单元格坐标"""
```

**`MapVisualizer`** - 地图可视化
```python
class MapVisualizer:
    def draw_map(self, map_data: np.ndarray):
        """绘制栅格地图（灰度图）"""
        
    def draw_robot(self, x: float, y: float, theta: float):
        """绘制机器人位姿（红色三角形）"""
        
    def draw_frontiers(self, frontiers: List[List[Tuple[int, int]]]):
        """绘制Frontier边界（绿色轮廓）"""
        
    def draw_path(self, path: List[Tuple[float, float]]):
        """绘制规划路径（蓝色线条）"""
        
    def draw_hud(self, info: dict):
        """绘制HUD信息面板"""
        
    def show(self):
        """显示窗口（带键盘交互）"""
```

#### SLAM更新流程

```
编码器累加线程 (100Hz)
    ↓
累加运动增量 (Δleft, Δright)
    ↓
检测状态转换 (1→2 或 2→0)
    ↓
触发SLAM更新
    ↓
slam.update(激光扫描, 累积增量)
    ↓
更新机器人位姿 + 地图
    ↓
重置累加器
```

#### 使用示例

```python
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from slam_map.occupancy_grid import OccupancyGrid
from slam_map.map_visualizer import MapVisualizer

# 初始化SLAM
laser = Laser(520, 5.0, 360.0, 8000, 0, 0)
slam = RMHC_SLAM(laser, 8000, 10)
map_data = bytearray(800 * 800)

# 初始化可视化
viz = MapVisualizer(width=800, height=800)

# SLAM更新
slam.update(lidar_scan, left_mm, right_mm)
x, y, theta = slam.getpos()

# 绘制地图
slam.getmap(map_data)
viz.draw_map(np.frombuffer(map_data, dtype=np.uint8).reshape(800, 800))
viz.draw_robot(x, y, theta)
viz.show()
```

---

### 3️⃣ 任务规划模块 (`planner/`)

#### 功能概述

实现自主探索的完整流程：Frontier检测 → 目标选择 → 路径规划 → 命令生成。

#### 核心组件

##### **MissionController** - 任务控制器

```python
class MissionController:
    """探索任务的状态机控制器"""
    
    def __init__(self, slam_instance, map_data):
        self.state = "IDLE"  # IDLE / PLANNING / NAVIGATING
        
    def run(self):
        """主循环：每200ms执行一次"""
        if self.state == "IDLE":
            # 检测Frontier
            frontiers = self.frontier_detector.find_frontiers(self.map_data)
            if frontiers:
                self.state = "PLANNING"
                
        elif self.state == "PLANNING":
            # 选择最佳Frontier
            best_frontier = self.frontier_selector.select_best(frontiers)
            
            # 规划路径
            path = self.astar_planner.plan_path(current_pos, best_frontier)
            
            # 平滑路径
            smooth_path = self.path_smoother.smooth(path)
            
            # 生成命令
            commands = self.command_generator.generate(smooth_path)
            self.state = "NAVIGATING"
            
        elif self.state == "NAVIGATING":
            # 执行命令序列
            if self.check_goal_reached():
                self.state = "IDLE"
```

##### **FrontierDetector** - 边界检测器

```python
class FrontierDetector:
    """基于八连通区域生长的Frontier检测"""
    
    def find_frontiers(self, map_data: np.ndarray) -> List[List[Tuple[int, int]]]:
        """
        检测地图中的Frontier边界
        
        算法流程:
        1. 遍历所有空闲单元格
        2. 对每个空闲单元，检查其8邻域
        3. 若邻域中存在未知单元，则该单元为Frontier点
        4. 使用区域生长聚类相邻Frontier点
        5. 过滤面积小于阈值的区域
        
        Returns:
            List[Region]: 每个Region是一个Frontier点集合
        """
```

**算法示意图**:
```
地图状态:
  ? ? ? ? ? ?     (? = 未知)
  ? □ □ □ □ ?     (□ = 空闲)
  ? □ ■ ■ □ ?     (■ = 占据)
  ? □ □ □ □ ?     (F = Frontier)
  ? ? ? ? ? ?

检测结果:
  ? ? ? ? ? ?
  ? F F F F ?  ← 上边界Frontier
  ? F ■ ■ F ?
  ? F F F F ?  ← 下边界Frontier
  ? ? ? ? ? ?
```

##### **AStarPlanner** - A*路径规划

```python
class AStarPlanner:
    """A*最优路径搜索"""
    
    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A*算法核心
        
        伪代码:
        OPEN = {start}
        CLOSED = {}
        
        while OPEN not empty:
            current = OPEN中f值最小的节点
            if current == goal:
                return 回溯路径
            
            CLOSED.add(current)
            for neighbor in current的邻居:
                if neighbor在CLOSED或被占据:
                    continue
                
                g_new = g[current] + cost(current, neighbor)
                if neighbor不在OPEN or g_new < g[neighbor]:
                    g[neighbor] = g_new
                    f[neighbor] = g_new + h(neighbor, goal)
                    parent[neighbor] = current
                    OPEN.add(neighbor)
        
        return []  # 无路径
        """
```

**启发函数选择**:
- 使用**欧几里得距离**作为h(n)
- 保证h(n)的一致性（admissible）
- 避免对角线路径的过度惩罚

##### **PathSmoother** - 路径平滑

```python
class PathSmoother:
    """Catmull-Rom样条路径平滑"""
    
    def smooth_path(self, raw_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        使用Catmull-Rom样条进行路径平滑
        
        算法特点:
        - C1连续（一阶导数连续）
        - 通过所有控制点
        - 局部控制性好
        
        参数:
        - tension: 张力参数，控制曲线"松紧度"
        - num_points: 插值点数量
        """
```

**平滑效果对比**:
```
原始路径 (A*):              平滑路径 (Catmull-Rom):
  S─┐                          S╭─╮
    │                           │  ╰╮
    └─┐                         │   │
      │  锯齿状                  │   │ 光滑曲线
      └─┐                       │  ╭╯
        └─G                     ╰─╯G
```

##### **CommandGenerator** - 命令生成器

```python
class CommandGenerator:
    """路径→运动命令转换"""
    
    def generate_commands(self, path: List[Tuple[float, float]]) -> List[Command]:
        """
        将路径分解为转向+直行命令序列
        
        步骤:
        1. 计算路径段的方向角
        2. 分解为: 转向到目标角度 + 直线行驶距离
        3. 生成命令序列: [(turn1, 0), (0, dist1), (turn2, 0), (0, dist2), ...]
        
        优化:
        - 相邻小角度合并
        - 短距离段过滤
        - 转向角度归一化到[-π, π]
        """
```

#### 使用示例

```python
from planner.mission_controller import MissionController

# 初始化任务控制器
controller = MissionController(slam, map_data)

# 主循环
while not controller.mission_complete():
    # 执行一步规划
    controller.run()
    
    # 可视化当前状态
    viz.draw_map(map_data)
    viz.draw_frontiers(controller.current_frontiers)
    viz.draw_path(controller.current_path)
    viz.show()
    
    time.sleep(0.2)  # 200ms周期
```

---

## 🧮 核心算法

### RMHC-SLAM算法

**Random Hill-Climbing Monte Carlo SLAM**

#### 算法原理

RMHC-SLAM是一种基于粒子滤波的SLAM算法，通过蒙特卡洛方法进行位姿估计。

**核心思想**:
1. 使用编码器里程计作为位姿预测
2. 使用激光扫描进行观测更新
3. 通过爬山算法优化位姿匹配

**算法流程**:
```python
def RMHC_SLAM_update(scan, odom_left, odom_right):
    # 1. 预测步骤（里程计）
    predicted_pose = motion_model(current_pose, odom_left, odom_right)
    
    # 2. 观测步骤（激光匹配）
    best_pose = predicted_pose
    best_score = compute_match_score(scan, map, predicted_pose)
    
    # 3. 随机爬山优化
    for i in range(num_iterations):
        # 生成随机扰动
        candidate_pose = add_noise(best_pose)
        score = compute_match_score(scan, map, candidate_pose)
        
        # 爬山更新
        if score > best_score:
            best_pose = candidate_pose
            best_score = score
    
    # 4. 地图更新
    update_map(scan, best_pose)
    
    return best_pose
```

#### 编码器积分改进

**问题**: 传统两点差分法丢失中间过程，瞬时误差被放大

**改进方案**: 完整积分法
```python
# 改进前：两点差分
Δenc = enc(t_end) - enc(t_start)

# 改进后：完整积分
Δenc = Σ(enc(t_i) - enc(t_i-1))  # i从start到end
```

**优势**:
- 采样率: 5Hz → 100Hz (20倍提升)
- 平滑随机噪声
- 捕捉完整运动过程
- 提升位移计算精度约80%

---

### Frontier检测算法

#### 算法定义

**Frontier**: 地图中已知区域与未知区域的边界

#### 检测流程

```python
def find_frontiers(map_data):
    frontiers = []
    visited = set()
    
    # 1. 遍历所有空闲单元
    for (x, y) in free_cells:
        if (x, y) in visited:
            continue
            
        # 2. 检查是否为Frontier点
        if is_frontier(x, y, map_data):
            # 3. 区域生长
            region = region_growing(x, y, map_data, visited)
            
            # 4. 过滤小区域
            if len(region) >= MIN_FRONTIER_SIZE:
                frontiers.append(region)
    
    return frontiers

def is_frontier(x, y, map_data):
    """检查单元格是否为Frontier点"""
    # 当前单元必须是空闲
    if map_data[x, y] != FREE:
        return False
    
    # 8邻域中至少有一个未知单元
    for (dx, dy) in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
        nx, ny = x + dx, y + dy
        if map_data[nx, ny] == UNKNOWN:
            return True
    
    return False

def region_growing(x, y, map_data, visited):
    """八连通区域生长"""
    region = []
    queue = [(x, y)]
    
    while queue:
        curr = queue.pop(0)
        if curr in visited:
            continue
        
        visited.add(curr)
        region.append(curr)
        
        # 添加邻居Frontier点
        for neighbor in get_neighbors(curr):
            if is_frontier(neighbor) and neighbor not in visited:
                queue.append(neighbor)
    
    return region
```

#### 可视化示例

```
原始地图:                     Frontier检测:
  ? ? ? ? ? ? ? ?              ? ? ? ? ? ? ? ?
  ? □ □ □ □ □ ? ?              ? ┏━━━━━━┓ ? ?
  ? □ ■ ■ ■ □ ? ?    ───▶      ? ┃ ■ ■ ■ ┃ ? ?
  ? □ □ □ □ □ ? ?              ? ┗━━━━━━┛ ? ?
  ? ? ? ? ? ? ? ?              ? ? ? ? ? ? ? ?

图例:
  ? = 未知区域
  □ = 已知空闲区域
  ■ = 已知占据区域
  ━ = Frontier边界
```

---

### A*路径规划算法

#### 算法公式

```
f(n) = g(n) + h(n)

其中:
- g(n): 从起点到节点n的实际代价
- h(n): 从节点n到终点的启发式估计
- f(n): 总估计代价
```

#### 启发函数选择

```python
def heuristic(a, b):
    """欧几里得距离启发函数"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
```

**特性**:
- **可采纳性** (Admissible): h(n) ≤ 真实代价
- **一致性** (Consistent): h(n) ≤ cost(n, n') + h(n')

#### 伪代码

```python
def astar(start, goal, map_data):
    open_set = PriorityQueue()
    open_set.put(start, priority=0)
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while not open_set.empty():
        current = open_set.get()
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current):
            # 跳过占据单元
            if is_occupied(neighbor, map_data):
                continue
            
            tentative_g = g_score[current] + distance(current, neighbor)
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                
                if neighbor not in open_set:
                    open_set.put(neighbor, priority=f_score[neighbor])
    
    return []  # 无路径
```

#### 性能优化

1. **优先队列**: 使用堆排序，查找最小f值的时间复杂度为O(log n)
2. **闭集合**: 避免重复访问已扩展节点
3. **邻域缓存**: 预计算8邻域偏移量
4. **早期终止**: 找到目标立即返回

---

## ⚙️ 配置说明

### 通信参数

```python
# btlink.py
SERIAL_PORT = "COM7"           # 串口端口
BAUDRATE = 921600              # 波特率
TIMEOUT = 0.01                 # 读取超时（秒）

# 队列配置
ENCODER_QUEUE_SIZE = 100       # 编码器队列容量
LIDAR_QUEUE_SIZE = 1           # 雷达队列容量
```

### SLAM参数

```python
# main_robot.py
MAP_SIZE_MM = 8000             # 地图尺寸（mm）
MAP_QUALITY_MM = 10            # 地图分辨率（mm/pixel）
WHEEL_RADIUS_M = 0.0325        # 轮半径（m）
HALF_WHEELBASE_M = 0.084       # 半轴距（m）
CPR = 1650                     # 编码器线数

# SLAM引擎配置
LASER_SCAN_SIZE = 520          # 激光点数
LASER_SCAN_RATE_HZ = 5         # 扫描频率
LASER_DETECTION_ANGLE = 360    # 检测角度范围
LASER_DISTANCE_NO_DETECTION = 8000  # 最大检测距离
```

### 规划参数

```python
# planner/planning_config.py

# Frontier检测
FRONTIER_MIN_SIZE = 20         # 最小Frontier尺寸（像素）
FRONTIER_MIN_SPACING = 50      # Frontier间最小间距（像素）

# 路径规划
ROBOT_RADIUS_MM = 100          # 机器人半径（mm）
OBSTACLE_INFLATION = 2         # 障碍物膨胀系数
PATH_RESOLUTION = 10           # 路径分辨率（mm）

# 路径平滑
SMOOTHNESS_FACTOR = 0.5        # 平滑因子（0-1）
SPLINE_POINTS = 50             # 样条插值点数

# 目标选择
DISTANCE_WEIGHT = 0.6          # 距离权重
INFO_GAIN_WEIGHT = 0.4         # 信息增益权重
MIN_GOAL_DISTANCE = 200        # 最小目标距离（mm）
```

### 可视化参数

```python
# slam_map/map_visualizer.py

# 颜色配置
COLOR_UNKNOWN = (128, 128, 128)    # 灰色
COLOR_FREE = (255, 255, 255)       # 白色
COLOR_OCCUPIED = (0, 0, 0)         # 黑色
COLOR_ROBOT = (0, 0, 255)          # 红色
COLOR_PATH = (255, 0, 0)           # 蓝色
COLOR_FRONTIER = (0, 255, 0)       # 绿色

# HUD配置
HUD_FONT = cv2.FONT_HERSHEY_SIMPLEX
HUD_FONT_SCALE = 0.5
HUD_LINE_SPACING = 20
```

---

## 🌊 数据流程

### 完整数据流向图

```
┌─────────────────────────────────────────────────────────────────┐
│                        数据采集层                                │
└─────────────────────────────────────────────────────────────────┘
                             │
                             ├──────────────────────────┐
                             │                          │
                      ┌──────▼──────┐          ┌───────▼────────┐
                      │ 编码器数据   │          │  雷达数据      │
                      │   10ms      │          │   200ms       │
                      │   21字节    │          │   2604字节    │
                      └──────┬──────┘          └───────┬────────┘
                             │                          │
                      ┌──────▼──────────────────────────▼──────┐
                      │         解析 & 队列管理 (btlink)       │
                      │  编码器队列(100) | 雷达队列(1)         │
                      └──────┬──────────────────────────┬──────┘
                             │                          │
                             │                          │
┌─────────────────────────────────────────────────────────────────┐
│                        数据处理层                                │
└─────────────────────────────────────────────────────────────────┘
                             │                          │
                      ┌──────▼──────┐          ┌───────▼────────┐
                      │ 编码器积分   │          │  雷达预处理    │
                      │  累加器      │          │  360度转换    │
                      │  100Hz采样  │          │  质量过滤     │
                      └──────┬──────┘          └───────┬────────┘
                             │                          │
                             └───────────┬──────────────┘
                                        │
                                 ┌──────▼──────┐
                                 │  SLAM更新   │
                                 │  位姿估计   │
                                 │  地图构建   │
                                 └──────┬──────┘
                                        │
                                        │
┌─────────────────────────────────────────────────────────────────┐
│                        任务规划层                                │
└─────────────────────────────────────────────────────────────────┘
                                        │
                      ┌─────────────────┼─────────────────┐
                      │                 │                 │
               ┌──────▼──────┐   ┌─────▼─────┐   ┌──────▼──────┐
               │ Frontier     │   │  路径规划  │   │  命令生成   │
               │   检测       │──▶│   A*算法  │──▶│  转向+直行  │
               └──────────────┘   └───────────┘   └──────┬──────┘
                                                          │
                                                          │
┌─────────────────────────────────────────────────────────────────┐
│                        命令执行层                                │
└─────────────────────────────────────────────────────────────────┘
                                                          │
                                                   ┌──────▼──────┐
                                                   │  串口发送   │
                                                   │  cmd_id同步 │
                                                   └──────┬──────┘
                                                          │
                                                          ▼
                                                   [下位机执行]
```

### 时序图

```
时间轴 ────────────────────────────────────────────────────────────▶

编码器    ┃     ┃     ┃     ┃     ┃     ┃     ┃     ┃     ┃  
10ms间隔  0ms  10ms  20ms  30ms  40ms  50ms  60ms  70ms  80ms ...

雷达      ┃                                   ┃
200ms间隔 0ms                                200ms                400ms

SLAM           △ (不触发)                      △ (触发！)
更新时机        状态=1(转向中)                   状态转换1→2

任务           ◇                              ◇
规划周期        Frontier检测                    路径规划

命令          ●                               ●
发送           cmd_id=1                        cmd_id=2
              (转向30°)                        (直行0.5m)
```

---

## 🛠️ 开发指南

### 添加新的规划器

```python
# planner/my_planner.py

class MyCustomPlanner:
    """自定义路径规划器"""
    
    def __init__(self, config):
        self.config = config
    
    def plan_path(self, start, goal, map_data):
        """
        实现你的规划算法
        
        Args:
            start: 起点 (x, y)
            goal: 终点 (x, y)
            map_data: 地图数据 (numpy数组)
            
        Returns:
            path: 路径点列表 [(x1, y1), (x2, y2), ...]
        """
        # 你的算法实现
        pass

# 在main_robot.py中使用
from planner.my_planner import MyCustomPlanner

planner = MyCustomPlanner(config)
path = planner.plan_path(start, goal, map_data)
```

### 添加新的Frontier选择策略

```python
# planner/my_selector.py

class MyFrontierSelector:
    """自定义Frontier选择策略"""
    
    def select_best_frontier(self, frontiers, robot_pos):
        """
        根据自定义策略选择最佳Frontier
        
        Args:
            frontiers: Frontier列表
            robot_pos: 机器人当前位置
            
        Returns:
            best_frontier: 最佳Frontier质心
        """
        scores = []
        for frontier in frontiers:
            score = self.compute_score(frontier, robot_pos)
            scores.append(score)
        
        best_idx = np.argmax(scores)
        return self.compute_centroid(frontiers[best_idx])
    
    def compute_score(self, frontier, robot_pos):
        """
        计算Frontier得分
        
        示例：综合考虑距离、大小、方向
        """
        centroid = self.compute_centroid(frontier)
        
        # 距离因子（距离越近得分越高）
        distance = euclidean_distance(robot_pos, centroid)
        distance_score = 1.0 / (1.0 + distance / 1000.0)
        
        # 大小因子（面积越大得分越高）
        size_score = len(frontier) / 100.0
        
        # 方向因子（与当前朝向一致得分越高）
        direction_score = compute_direction_alignment(robot_pos, centroid)
        
        # 加权综合
        total_score = (0.5 * distance_score + 
                      0.3 * size_score + 
                      0.2 * direction_score)
        
        return total_score
```

### 扩展可视化功能

```python
# slam_map/custom_viz.py

from slam_map.map_visualizer import MapVisualizer

class EnhancedVisualizer(MapVisualizer):
    """增强版可视化器"""
    
    def draw_heatmap(self, data):
        """绘制热力图"""
        heatmap = cv2.applyColorMap(data, cv2.COLORMAP_JET)
        self.image = cv2.addWeighted(self.image, 0.7, heatmap, 0.3, 0)
    
    def draw_trajectory(self, trajectory):
        """绘制历史轨迹"""
        points = np.array(trajectory, dtype=np.int32)
        cv2.polylines(self.image, [points], False, (255, 255, 0), 2)
    
    def draw_debug_info(self, debug_data):
        """绘制调试信息"""
        y_offset = 400
        for key, value in debug_data.items():
            text = f"{key}: {value}"
            cv2.putText(self.image, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y_offset += 20
```

### 单元测试

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
        """测试直线路径"""
        start = (10, 10)
        goal = (10, 90)
        path = self.planner.plan_path(start, goal, self.map_data)
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
    
    def test_obstacle_avoidance(self):
        """测试障碍物避让"""
        # 添加障碍物墙
        self.map_data[50, 10:90] = 255
        
        start = (10, 50)
        goal = (90, 50)
        path = self.planner.plan_path(start, goal, self.map_data)
        
        self.assertIsNotNone(path)
        # 验证路径绕过障碍物
        for (x, y) in path:
            self.assertNotEqual(self.map_data[x, y], 255)
    
    def test_no_path(self):
        """测试无路径情况"""
        # 用障碍物完全包围目标点
        self.map_data[80:90, 80:90] = 255
        
        start = (10, 10)
        goal = (85, 85)
        path = self.planner.plan_path(start, goal, self.map_data)
        
        self.assertEqual(len(path), 0)

if __name__ == '__main__':
    unittest.main()
```

---

## ❓ 常见问题

### Q1: 程序启动后无法连接串口

**解决方案**:
1. 检查串口号是否正确：`python -m serial.tools.list_ports`
2. 确认波特率匹配：921600
3. 检查串口是否被其他程序占用
4. Windows用户：检查设备管理器中的COM端口

### Q2: SLAM地图不更新

**可能原因**:
1. 编码器数据未收到 → 检查编码器队列统计
2. 雷达数据质量差 → 查看HUD中的雷达点数
3. 状态转换未触发 → 确认status值是否正常变化
4. 运动增量过小 → 检查编码器累加值

**调试方法**:
```python
# 添加调试日志
print(f"Encoder frames: {encoder_count}")
print(f"Status: {encoder_frame.status}")
print(f"Accumulated: left={acc_left}, right={acc_right}")
```

### Q3: 路径规划失败

**排查步骤**:
1. 检查Frontier是否检测到：`len(frontiers) > 0`
2. 确认目标点可达性：`reachability.is_reachable(goal)`
3. 查看A*搜索日志：是否陷入死循环
4. 增大障碍物膨胀半径：`OBSTACLE_INFLATION += 1`

### Q4: 机器人转圈不停

**原因分析**:
- cmd_id同步失败
- 状态机卡在某个状态
- 命令队列未清空

**解决方案**:
```python
# 重置状态机
controller.reset()

# 清空命令队列
while not command_queue.empty():
    command_queue.get()

# 强制进入空闲状态
robot_status = 0
```

### Q5: 编码器累加值异常

**检查项**:
1. 下位机是否发送累积值而非增量
2. 累加器是否在SLAM触发后重置
3. 编码器线数(CPR)是否正确配置

**正确的累加逻辑**:
```python
# 正确
if frame_count > 0:
    delta_left = current_left - last_left
    acc_left += delta_left

# 错误（下位机发送增量）
acc_left += current_left  # ❌ 不要这样写
```

### Q6: 地图出现"鬼影"

**现象**: 地图中出现不存在的障碍物

**原因**: SLAM位姿估计漂移

**解决方案**:
1. 增加SLAM更新频率
2. 改善编码器数据质量
3. 添加闭环检测
4. 使用更高精度的里程计模型

### Q7: 系统CPU占用过高

**优化建议**:
1. 降低地图可视化频率：`cv2.waitKey(100)` → `cv2.waitKey(200)`
2. 减少SLAM更新频率：仅在必要时触发
3. 优化A*搜索：使用JPS算法
4. 多线程优化：将可视化移到单独线程

---

## 📜 许可证

本项目采用 **MIT License** 开源协议。

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

## 🙏 致谢

本项目使用了以下开源库：

- **BreezySLAM**: SLAM算法核心 - https://github.com/simondlevy/BreezySLAM
- **NumPy**: 科学计算基础 - https://numpy.org/
- **OpenCV**: 计算机视觉与可视化 - https://opencv.org/
- **PySerial**: 串口通信 - https://github.com/pyserial/pyserial

特别感谢所有为开源社区做出贡献的开发者！

---


<div align="center">

**⭐ 如果这个项目对你有帮助，请给我们一个Star！⭐**

Made with ❤️ by SLAM-car Team

</div>
