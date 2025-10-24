"""
main_robot_improved.py - 改进版机器人导航主控制程序

核心改进：
1. 编码器积分累加：10ms采样，两次SLAM更新之间累积所有增量
2. 双帧分离处理：编码器帧（10ms）+ 雷达帧（200ms）
3. SLAM触发条件不变：状态转换时（1→2, 2→0）
4. 使用累积的编码器增量计算位移，而非两点差分

使用方法：
    python main_robot_improved.py --port COM7 --baud 921600
"""
import threading
import time
import argparse
import logging
from datetime import datetime
from pathlib import Path
from typing import Optional

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

from slam_map import OccupancyGrid
from planner import (MissionController, StatusReport, Phase,
                    Position, WheelConfig, SpeedLimits, Tolerance)

# 导入改进版通信模块
import sys
sys.path.insert(0, '/home/claude')
from btlink import BTLink, WheelGeom, EncoderFrame, LidarFrame, build_scan360

# 地图配置
GRID_SIZE_PIXELS = 800
WORLD_SIZE_METERS = 6
RANDOM_SEED = 9999

current_position = Position(0, 0, 0)


class DataLogger:
    """数据记录器"""

    def __init__(self, filename: str = None):
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_data_improved_{timestamp}.txt"

        self.file_path = Path(filename)
        self.frame_count = 0

        with open(self.file_path, 'w', encoding='utf-8') as f:
            f.write("=" * 100 + "\n")
            f.write("机器人导航数据日志 (改进版 - 编码器积分)\n")
            f.write(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 100 + "\n\n")

    def log_slam_update(self, update_info: dict):
        """记录SLAM更新事件"""
        self.frame_count += 1

        with open(self.file_path, 'a', encoding='utf-8') as f:
            f.write("┌" + "─" * 98 + "┐\n")
            f.write(f"│ SLAM更新 #{self.frame_count:<5} " + " " * 80 + "│\n")
            f.write(f"│ 时间: {datetime.now().strftime('%H:%M:%S.%f')[:-3]:<20} " + " " * 67 + "│\n")
            f.write("├" + "─" * 98 + "┤\n")

            # 编码器累积信息
            f.write("│ [编码器积分] " + " " * 82 + "│\n")
            f.write(f"│   累积帧数: {update_info['encoder_frames']:<5}  "
                   f"时间跨度: {update_info['time_span']:.3f}s  "
                   f"采样率: {update_info['sample_rate']:.1f}Hz" + " " * 20 + "│\n")
            f.write(f"│   累积增量: ΔL={update_info['sum_dL']:<8}  ΔR={update_info['sum_dR']:<8}  "
                   f"比例={update_info['ratio']:.3f}" + " " * 20 + "│\n")
            f.write("│" + " " * 98 + "│\n")

            # 运动增量
            f.write("│ [运动增量] " + " " * 84 + "│\n")
            f.write(f"│   距离: {update_info['distance_mm']:.1f} mm  "
                   f"角度: {update_info['angle_deg']:.2f}°  "
                   f"时间: {update_info['dt_s']:.3f}s" + " " * 25 + "│\n")
            f.write("│" + " " * 98 + "│\n")

            # 机器人位姿
            f.write("│ [位姿更新] " + " " * 84 + "│\n")
            f.write(f"│   位置: X={update_info['pos_x']:>7.3f}m  Y={update_info['pos_y']:>7.3f}m  "
                   f"朝向={update_info['heading']:>6.1f}°" + " " * 30 + "│\n")

            f.write("└" + "─" * 98 + "┘\n\n")

    def log_summary(self, stats: dict):
        """记录会话摘要"""
        with open(self.file_path, 'a', encoding='utf-8') as f:
            f.write("\n" + "=" * 100 + "\n")
            f.write("会话摘要\n")
            f.write(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"SLAM更新次数: {self.frame_count}\n")
            f.write(f"编码器帧总数: {stats.get('encoder_frames', 0)}\n")
            f.write(f"雷达帧总数: {stats.get('lidar_frames', 0)}\n")
            f.write(f"编码器帧丢弃: {stats.get('encoder_dropped', 0)}\n")
            f.write("=" * 100 + "\n")


class EncoderAccumulator:
    """编码器增量累加器"""

    def __init__(self):
        self.sum_dL = 0          # 累积左轮增量
        self.sum_dR = 0          # 累积右轮增量
        self.sum_dt_us = 0       # 累积时间(微秒)
        self.frame_count = 0     # 累积帧数

        self.last_encoder_l: Optional[int] = None
        self.last_encoder_r: Optional[int] = None
        self.last_time_us: Optional[int] = None

        self.first_time_us: Optional[int] = None  # 累积周期起始时间

    def reset(self):
        """重置累加器（SLAM更新后调用）"""
        self.sum_dL = 0
        self.sum_dR = 0
        self.sum_dt_us = 0
        self.frame_count = 0
        self.first_time_us = None
        # 注意：不重置last_encoder_*，保持连续性

    def accumulate(self, frame: EncoderFrame):
        """累加一帧编码器数据"""
        # 第一帧，只记录初始值
        if self.last_encoder_l is None:
            self.last_encoder_l = frame.encoder_l
            self.last_encoder_r = frame.encoder_r
            self.last_time_us = frame.time_us
            self.first_time_us = frame.time_us
            return

        # 计算增量
        dL = frame.encoder_l - self.last_encoder_l
        dR = frame.encoder_r - self.last_encoder_r
        dt_us = frame.time_us - self.last_time_us

        # 累加
        self.sum_dL += dL
        self.sum_dR += dR
        self.sum_dt_us += dt_us
        self.frame_count += 1

        # 更新上一帧
        self.last_encoder_l = frame.encoder_l
        self.last_encoder_r = frame.encoder_r
        self.last_time_us = frame.time_us

    def compute_motion(self, geom: WheelGeom) -> tuple:
        """
        计算累积的运动增量
        返回: (dxy_mm, dtheta_deg, dt_s)
        """
        if self.frame_count == 0:
            return (0.0, 0.0, 1e-6)

        # 转换为轮子转过的圈数
        revL = self.sum_dL / float(geom.ticks_per_rev)
        revR = self.sum_dR / float(geom.ticks_per_rev)

        # 计算行驶距离
        import math
        sL = (2.0 * math.pi * geom.wheel_radius_m) * revL
        sR = (2.0 * math.pi * geom.wheel_radius_m) * revR

        ds = 0.5 * (sL + sR)  # 平均距离(米)
        dtheta_rad = (sR - sL) / (2.0 * geom.half_wheelbase_m)

        dt_s = self.sum_dt_us / 1e6

        # 转换单位
        dxy_mm = ds * 1000.0
        dtheta_deg = dtheta_rad * 180.0 / math.pi

        return (dxy_mm, dtheta_deg, dt_s)

    def get_stats(self) -> dict:
        """获取累加器统计信息"""
        ratio = self.sum_dR / self.sum_dL if self.sum_dL != 0 else 0
        time_span = self.sum_dt_us / 1e6 if self.sum_dt_us > 0 else 0
        sample_rate = self.frame_count / time_span if time_span > 0 else 0

        return {
            'encoder_frames': self.frame_count,
            'sum_dL': self.sum_dL,
            'sum_dR': self.sum_dR,
            'ratio': ratio,
            'time_span': time_span,
            'sample_rate': sample_rate
        }


class NavigationThread(threading.Thread):
    """改进版导航控制线程"""

    def __init__(self, serial_link: BTLink, wheel_params: WheelGeom,
                 slam_engine: RMHC_SLAM, grid_map: OccupancyGrid,
                 mission_controller: MissionController, position_data: list,
                 map_buffer: bytearray, max_radar_range: int,
                 status_container: dict, data_logger: DataLogger = None):
        super().__init__(daemon=True)

        self.serial = serial_link
        self.wheels = wheel_params
        self.slam = slam_engine
        self.map = grid_map
        self.mission = mission_controller
        self.position = position_data
        self.map_buffer = map_buffer
        self.max_range = max_radar_range
        self.status_report = status_container
        self.logger = data_logger

        self._stop_event = threading.Event()

        # 编码器累加器（核心改进）
        self.encoder_acc = EncoderAccumulator()

        # 控制状态
        self.cmd_active = False
        self.cmd_counter = 0
        self.last_robot_status = None

        # 两步运动
        self.first_run = True
        self.step_counter = 0
        self.planned_turn = None
        self.planned_distance = None

        # 雷达数据缓存
        self.latest_lidar_scan: Optional[List[int]] = None

        # 重试机制
        self.retry_counter = 0
        self.max_retries = 5
        self.retry_interval = 10

    def stop(self):
        """停止线程"""
        self._stop_event.set()

    def _get_status_text(self, status_code: int) -> str:
        """状态码转文本"""
        status_map = {0: "空闲", 1: "转向中", 2: "直行中"}
        return status_map.get(status_code, f"未知({status_code})")

    def run(self):
        """主控制循环"""
        print("\n" + "="*100)
        print("🚀 改进版导航系统启动")
        print("📊 编码器积分模式：10ms采样，完整累加")
        print("="*100 + "\n")

        # 启动两个处理循环
        encoder_thread = threading.Thread(target=self._encoder_loop, daemon=True)
        encoder_thread.start()

        # 主循环：处理雷达和SLAM
        while not self._stop_event.is_set():
            # 非阻塞获取雷达帧
            lidar_frame = self.serial.get_lidar_frame(timeout=0.3)

            if lidar_frame:
                # 更新雷达扫描缓存
                self.latest_lidar_scan = build_scan360(
                    lidar_frame.points,
                    max_range_mm=self.max_range,
                    quality_min=0
                )
                print(f"📡 收到雷达帧: {lidar_frame.data_count}点")

            # 获取最新的编码器帧（用于状态判断）
            encoder_frame = self.serial.get_encoder_frame(timeout=0.05)

            if not encoder_frame:
                continue

            # ========== 状态转换检测 ==========
            should_update_slam = False

            # 转向完成→直行开始 (1→2)
            if encoder_frame.status == 2 and self.last_robot_status == 1:
                should_update_slam = True
                self.last_robot_status = 2
                print("🔄 状态转换: 转向→直行")

            # 直行完成→空闲 (2→0)
            if encoder_frame.status == 0 and self.last_robot_status == 2:
                should_update_slam = True
                self.last_robot_status = 0
                print("🔄 状态转换: 直行→空闲")

            # 第一帧强制更新
            if self.first_run:
                should_update_slam = True
                self.first_run = False

            # ========== SLAM更新（使用累积增量）==========
            if should_update_slam:
                if self.latest_lidar_scan is None:
                    print("⚠️  无可用雷达数据，跳过SLAM更新")
                    continue

                # 检查累加器数据
                acc_stats = self.encoder_acc.get_stats()
                if acc_stats['encoder_frames'] == 0:
                    print("⚠️  累加器为空，跳过SLAM更新")
                    continue

                print(f"\n{'='*100}")
                print(f"🎯 SLAM更新触发 | 累积帧数: {acc_stats['encoder_frames']} | "
                      f"时间跨度: {acc_stats['time_span']:.3f}s")
                print(f"   编码器增量: ΔL={acc_stats['sum_dL']} ΔR={acc_stats['sum_dR']} "
                      f"(比例={acc_stats['ratio']:.3f})")

                # 计算累积的运动增量
                start_time = time.time()
                motion_increment = self.encoder_acc.compute_motion(self.wheels)

                print(f"   运动增量: 距离={motion_increment[0]:.1f}mm "
                      f"角度={motion_increment[1]:.2f}° "
                      f"时间={motion_increment[2]:.3f}s")

                # 执行SLAM更新
                self.slam.update(self.latest_lidar_scan, motion_increment)
                self.position[0], self.position[1], self.position[2] = self.slam.getpos()
                self.slam.getmap(self.map_buffer)
                self.map.update_from_slam(self.map_buffer)
                self.map.update_robot_pose(
                    self.position[0] / 1000.0,
                    self.position[1] / 1000.0,
                    self.position[2]
                )

                current_position.x_meters = self.position[0] / 1000.0
                current_position.y_meters = self.position[1] / 1000.0
                current_position.heading_degrees = self.position[2]

                elapsed = time.time() - start_time
                print(f"✅ SLAM更新完成: {elapsed:.3f}秒")
                print(f"   位姿: ({current_position.x_meters:.3f}m, "
                      f"{current_position.y_meters:.3f}m, {current_position.heading_degrees:.1f}°)")
                print(f"{'='*100}\n")

                # 记录日志
                if self.logger:
                    log_info = {
                        **acc_stats,
                        'distance_mm': motion_increment[0],
                        'angle_deg': motion_increment[1],
                        'dt_s': motion_increment[2],
                        'pos_x': current_position.x_meters,
                        'pos_y': current_position.y_meters,
                        'heading': current_position.heading_degrees
                    }
                    self.logger.log_slam_update(log_info)

                # 重置累加器
                self.encoder_acc.reset()

            # ========== 命令发送逻辑 ==========
            if encoder_frame.status == 0 and encoder_frame.cmd_id == self.cmd_counter:
                if self.cmd_active:
                    self.cmd_active = False
                    self.retry_counter = 0
                    print("✓ 命令执行完成")

                if not self.cmd_active:
                    # 获取新命令
                    if self.step_counter == 0:
                        action = self.mission.compute_next_action(current_position)
                        self.planned_turn, self.planned_distance, self.status_report["report"] = action

                    self.last_robot_status = 1
                    self.cmd_counter += 1

                    # 两步策略
                    if self.step_counter == 0:
                        # 步骤1: 转向
                        self.serial.send_cmd(
                            self.cmd_counter,
                            turn_rad=self.planned_turn,
                            distance_m=0
                        )
                        print(f"→ 转向命令: {self.planned_turn:.2f}° (ID={self.cmd_counter})")
                        self.step_counter += 1
                    elif self.step_counter == 1:
                        # 步骤2: 直行
                        self.serial.send_cmd(
                            self.cmd_counter,
                            turn_rad=0,
                            distance_m=self.planned_distance
                        )
                        print(f"→ 直行命令: {self.planned_distance:.3f}m (ID={self.cmd_counter})")
                        self.step_counter = 0

                    self.cmd_active = True

    def _encoder_loop(self):
        """独立线程：持续累加编码器数据"""
        print("📊 编码器累加线程启动\n")

        while not self._stop_event.is_set():
            # 获取编码器帧（阻塞等待）
            enc_frame = self.serial.get_encoder_frame(timeout=0.1)

            if enc_frame:
                # 累加到累加器
                self.encoder_acc.accumulate(enc_frame)

                # 定期输出状态（每10帧）
                if self.encoder_acc.frame_count % 10 == 0:
                    stats = self.encoder_acc.get_stats()
                    print(f"📊 累加器: {stats['encoder_frames']}帧 | "
                          f"ΔL={stats['sum_dL']:>6} ΔR={stats['sum_dR']:>6} | "
                          f"比例={stats['ratio']:.3f} | "
                          f"{stats['sample_rate']:.1f}Hz")


def main():
    parser = argparse.ArgumentParser(description="改进版机器人导航系统 (编码器积分)")
    parser.add_argument("--port", type=str, default="COM7", help="串口名称")
    parser.add_argument("--baud", type=int, default=921600, help="波特率")

    # 轮子参数
    parser.add_argument("--wheel-radius", type=float, default=0.0325,
                       help="轮子半径(米)")
    parser.add_argument("--half-wheelbase", type=float, default=0.084,
                       help="半轴距(米)")
    parser.add_argument("--cpr", type=int, default=1650,
                       help="编码器CPR")

    # 日志选项
    parser.add_argument("--log-file", type=str, default=None, help="日志文件名")
    parser.add_argument("--no-log", action="store_true", help="禁用日志")

    args = parser.parse_args()

    print("\n" + "="*100)
    print("🔧 机器人导航系统")
    print("="*100 + "\n")

    # 初始化日志
    logger = None if args.no_log else DataLogger(args.log_file)
    if logger:
        print(f"📝 数据记录到: {logger.file_path}\n")

    # 初始化通信（改进版）
    wheel_geom = WheelGeom(args.wheel_radius, args.half_wheelbase, args.cpr)
    serial_link = BTLink(
        port=args.port,
        baud=args.baud,
        encoder_queue_size=100,  # 存储1秒编码器数据
        lidar_queue_size=1,      # 雷达只保留最新
        logger=logging.getLogger("BTLink")
    )
    serial_link.start()

    # 初始化SLAM（520点雷达）
    map_data = bytearray(GRID_SIZE_PIXELS * GRID_SIZE_PIXELS)
    laser_model = Laser(520, 5.0, 360.0, 8000, 0, 0)
    slam_engine = RMHC_SLAM(
        laser_model, GRID_SIZE_PIXELS, WORLD_SIZE_METERS,
        hole_width_mm=100
    )

    # 初始化地图
    occupancy_grid = OccupancyGrid(GRID_SIZE_PIXELS, WORLD_SIZE_METERS)
    position_state = [0, 0, 0]
    occupancy_grid.update_robot_pose(0.0, 0.0, 0.0)
    # 初始化任务控制器
    robot_wheels = WheelConfig(
        radius_mm=args.wheel_radius * 1000,
        half_baseline_mm=args.half_wheelbase * 1000,
        encoder_ticks_per_revolution=args.cpr
    )
    target_coords = (2.3, 2.3)
    controller = MissionController(
        occupancy_grid, robot_wheels,
        target_world=target_coords,
        speed_limits=SpeedLimits(),
        tolerance=Tolerance()
    )

    # 初始状态
    initial_status = StatusReport(
        current_phase=Phase.EXPLORING,
        robot_position=Position(0.0, 0.0, 0.0),
        target_waypoint=None,
        detected_frontiers=0,
        tracked_region=None,
        goal_discovered=False,
        path_length=0,
        status_message="系统初始化中"
    )
    status_container = {"report": initial_status}

    # 启动导航线程
    navigation_thread = NavigationThread(
        serial_link, wheel_geom, slam_engine, occupancy_grid,
        controller, position_state, map_data,
        max_radar_range=8000, status_container=status_container,
        data_logger=logger
    )
    navigation_thread.start()

    # 可视化循环
    print("\n" + "="*100)
    print("🚀 系统已启动 (改进版)")
    print("="*100 + "\n")

    try:
        while True:
            status = status_container.get("report", None)
            robot_pose = occupancy_grid.get_robot_pose()

            # 生成HUD文本
            hud_text = ""
            if status is not None:
                phase_name = {
                    Phase.EXPLORING: "探索中",
                    Phase.RETURNING: "返回中",
                    Phase.NAVIGATING_TO_GOAL: "前往目标",
                    Phase.COMPLETED: "已完成"
                }.get(status.current_phase, status.current_phase.name)

                # 添加通信统计
                comm_stats = serial_link.get_stats()

                hud_text = (
                    f"阶段: {phase_name}\n"
                    f"边界点: {status.detected_frontiers}\n"
                    f"区域: {status.tracked_region}\n"
                    f"路径: {status.path_length}节点\n"
                    f"目标: {'已发现' if status.goal_discovered else '搜索中'}\n"
                    f"状态: {status.status_message}\n"
                    f"───────────────────\n"
                    f"编码器帧: {comm_stats['encoder_frames']}\n"
                    f"雷达帧: {comm_stats['lidar_frames']}\n"
                    f"丢帧: E={comm_stats['encoder_dropped']} L={comm_stats['lidar_dropped']}"
                )

            # 渲染
            frontier_list = status.frontier_list if status else None
            best_frontier = status.selected_frontier if status else None
            sample_points = status.sampled_frontiers if status else None

            occupancy_grid.renderer.display(
                occupancy_grid.data, robot_pose, extra_text=hud_text,
                roi_rect=(status.tracked_region if status else None),
                frontier_points=frontier_list,
                best_frontier_point=best_frontier,
                spatially_sampled=sample_points,
                exit_pose_pix=status.goal_position if status else None
            )

    except KeyboardInterrupt:
        print("\n⚠️  正在关闭系统...")
        if logger:
            comm_stats = serial_link.get_stats()
            logger.log_summary(comm_stats)
            print(f"✓ 数据已保存: {logger.file_path}")

    finally:
        navigation_thread.stop()
        serial_link.stop()
        print("\n✓ 系统已关闭\n")


if __name__ == "__main__":
    main()
