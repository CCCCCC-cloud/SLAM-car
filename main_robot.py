"""
机器人导航主控制程序 (修复版)
修复内容:
1. 修复SLAM更新逻辑错误 (BUG-1)
2. 统一轮子参数配置 (BUG-2)
3. 添加编码器数据合理性检查
4. 改进激光雷达数据填充策略
"""
import threading
import time
import argparse
import logging
from datetime import datetime
from pathlib import Path

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser

from slam_map import OccupancyGrid
from planner import (MissionController, StatusReport, Phase,
                    Position, WheelConfig, SpeedLimits, Tolerance)
from serial_io import BTLink, WheelGeom as BTWheels, to_slam_inputs

# 地图配置
GRID_SIZE_PIXELS = 800  # 网格大小(像素)
WORLD_SIZE_METERS = 6   # 世界大小(米)
RANDOM_SEED = 9999      # 随机种子

current_position = Position(0, 0, 0)  # 当前位置


class DataLogger:
    """数据记录器 - 格式化输出"""

    def __init__(self, filename: str = None):
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_data_{timestamp}.txt"

        self.file_path = Path(filename)
        self.frame_count = 0  # 帧计数器

        # 创建文件并写入头部信息
        with open(self.file_path, 'w', encoding='utf-8') as f:
            f.write("=" * 100 + "\n")
            f.write("机器人导航数据日志\n")
            f.write(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 100 + "\n\n")

    def log_frame_data(self, frame_info: dict):
        """记录一帧数据"""
        self.frame_count += 1

        with open(self.file_path, 'a', encoding='utf-8') as f:
            # 帧头部
            f.write("┌" + "─" * 98 + "┐\n")
            f.write(f"│ 帧 {self.frame_count:<5} " + " " * 85 + "│\n")
            f.write(f"│ 时间戳: {datetime.now().strftime('%H:%M:%S.%f')[:-3]:<20} " + " " * 67 + "│\n")
            f.write("├" + "─" * 98 + "┤\n")

            # 机器人状态
            f.write("│ [机器人状态] " + " " * 82 + "│\n")
            f.write(f"│   位置: X={frame_info['pos_x']:>7.3f}m  Y={frame_info['pos_y']:>7.3f}m  "
                   f"朝向={frame_info['heading']:>6.1f}°" + " " * 30 + "│\n")
            f.write(f"│   左编码器: {frame_info['encoder_l']:>8}  右编码器: {frame_info['encoder_r']:>8}  "
                   f"偏航角: {frame_info['yaw']:>7.2f}°" + " " * 15 + "│\n")
            f.write(f"│   命令ID: {frame_info['cmd_id']:<5}  状态: {frame_info['status_text']:<20}"
                   + " " * 45 + "│\n")
            f.write("│" + " " * 98 + "│\n")

            # 移动增量
            if frame_info.get('has_movement'):
                f.write("│ [移动增量] " + " " * 84 + "│\n")
                f.write(f"│   距离: {frame_info['distance_inc']:>7.1f} mm  "
                       f"角度: {frame_info['angle_inc']:>6.2f}°  "
                       f"时间: {frame_info['time_inc']:>6.3f} 秒" + " " * 25 + "│\n")
                f.write("│" + " " * 98 + "│\n")

            # 雷达数据统计
            f.write("│ [激光扫描] " + " " * 80 + "│\n")
            f.write(f"│   总点数: {frame_info['radar_points']:<5}  "
                   f"有效点数: {frame_info['valid_points']:<5}  "
                   f"范围: [{frame_info['min_dist']:.0f}, {frame_info['max_dist']:.0f}] mm" + " " * 15 + "│\n")

            # 雷达数据采样(每30度一个样本)
            f.write("│" + " " * 98 + "│\n")
            f.write("│   角度分布(每30度采样):" + " " * 62 + "│\n")

            scan_data = frame_info.get('scan_samples', {})
            if scan_data:
                for angle in range(-180, 180, 30):
                    distance = scan_data.get(angle, 0)
                    bar_length = min(40, int(distance / 200))  # 每200mm一个字符
                    bar = "█" * bar_length
                    f.write(f"│   {angle:>4}°: {distance:>5.0f}mm │{bar:<40}│" + " " * (40 - len(bar) + 2) + "│\n")

            f.write("│" + " " * 98 + "│\n")

            # 任务状态
            if frame_info.get('mission_status'):
                f.write("│ [任务状态] " + " " * 84 + "│\n")
                mission = frame_info['mission_status']
                f.write(f"│   阶段: {mission['phase']:<20}  边界点: {mission['frontiers']:<5}" + " " * 35 + "│\n")
                f.write(f"│   消息: {mission['message']:<80}" + " " * 5 + "│\n")

            # 帧尾部
            f.write("└" + "─" * 98 + "┘\n\n")

    def log_summary(self):
        """记录会话摘要"""
        with open(self.file_path, 'a', encoding='utf-8') as f:
            f.write("\n" + "=" * 100 + "\n")
            f.write("会话摘要\n")
            f.write(f"结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"记录的总帧数: {self.frame_count}\n")
            f.write("=" * 100 + "\n")


class NavigationThread(threading.Thread):
    """带两步命令和数据记录的机器人主控制循环 (修复版)"""

    def __init__(self, serial_link: BTLink, wheel_params: BTWheels,
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
        self._stop_event = threading.Event()
        self.logger = data_logger

        # 控制状态
        self.cmd_active = False
        self.last_encoder_state = None
        self.cmd_counter = 0
        self.status_report = status_container
        self.last_robot_status = None

        # 两步状态
        self.first_run = True
        self.step_counter = 0
        self.planned_turn = None
        self.planned_distance = None

        # 仅第一帧超时重传
        self.first_frame_sent = False
        self.first_frame_timeout = 0
        self.MAX_FIRST_FRAME_RETRIES = 3

        # 命令重发机制
        self.retry_counter = 0
        self.max_retries = 5
        self.retry_interval = 10

    def stop(self):
        """停止线程"""
        self._stop_event.set()

    def _get_status_text(self, status_code: int) -> str:
        """将状态码转换为文本"""
        status_map = {
            0: "空闲(就绪)",
            1: "转向中",
            2: "直行中"
        }
        return status_map.get(status_code, f"未知({status_code})")

    def _parse_scan_samples(self, scan_360: list, sample_interval: int = 30) -> dict:
        """提取雷达扫描样本数据"""
        samples = {}
        for angle in range(-180, 180, sample_interval):
            index = angle + 180
            if 0 <= index < len(scan_360):
                samples[angle] = scan_360[index]
        return samples

    def _check_encoder_sanity(self, frame) -> bool:
        """
        【新增】检查编码器数据合理性
        检测异常的左右编码器比例
        """
        if self.last_encoder_state is None:
            return True  # 第一帧，无法比较

        prev_l, prev_r, _ = self.last_encoder_state
        curr_l, curr_r = frame.encoder_l, frame.encoder_r

        delta_l = abs(curr_l - prev_l)
        delta_r = abs(curr_r - prev_r)

        # 如果两个编码器都没变化，正常
        if delta_l == 0 and delta_r == 0:
            return True

        # 如果只有一个变化，可能异常（除非原地转）
        if delta_l == 0 or delta_r == 0:
            print(f"⚠️  编码器异常: 只有一侧变化 L={delta_l} R={delta_r}")
            return True  # 暂时允许，因为可能是原地转

        # 检查比例是否合理 (应该在 0.5 ~ 2.0 之间)
        ratio = delta_r / delta_l
        if ratio < 0.3 or ratio > 3.0:
            print(f"❌ 编码器比例异常: L={delta_l} R={delta_r} 比例={ratio:.2f}")
            return False

        return True

    def run(self):
        """主控制循环"""
        while not self._stop_event.is_set():
            # 每次循环读取一帧
            frame = self.serial.get_frame(timeout=1.0)

            # 处理第一帧超时重传
            if not frame and not self.first_frame_sent:
                self.first_frame_timeout += 1
                print(f"⚠️  超时 {self.first_frame_timeout}/{self.MAX_FIRST_FRAME_RETRIES}")

                if self.first_frame_timeout >= self.MAX_FIRST_FRAME_RETRIES:
                    print("❌ 达到最大重传次数，停止系统")
                    break

                # 使用当前命令参数重发第一帧
                if self.step_counter == 1:
                    self.serial.send_cmd(
                        self.cmd_counter,
                        turn_rad=self.planned_turn,
                        distance_m=0
                    )
                else:
                    self.serial.send_cmd(
                        self.cmd_counter,
                        turn_rad=0,
                        distance_m=self.planned_distance
                    )
                continue

            # 收到第一帧后重置超时计数器
            if frame and not self.first_frame_sent:
                self.first_frame_sent = True
                print("✓ 成功接收第一帧")

            if not frame:
                continue  # 第一帧之后忽略超时

            # 控制台输出
            print("=" * 100)
            print(f"[接收帧] 命令={frame.cmd_id} | 状态={self._get_status_text(frame.status)} | "
                  f"点数={len(frame.points)} | 编码器=({frame.encoder_l}, {frame.encoder_r})")
            print("=" * 100)

            # 【修复BUG-1】移除无条件的 should_update_slam = True
            should_update_slam = False

            # 检测状态转换(1→2或2→0)
            if frame.status == 2 and self.last_robot_status == 1:
                should_update_slam = True
                self.last_robot_status = 2
                print("🔄 状态转换: 转向中→直行中")

            if frame.status == 0 and self.last_robot_status == 2:
                should_update_slam = True
                self.last_robot_status = 0
                print("🔄 状态转换: 直行中→空闲")

            # 第一帧强制更新
            if self.first_run:
                should_update_slam = True

            # 【新增】编码器合理性检查
            if not self._check_encoder_sanity(frame):
                print("⚠️  编码器数据异常，跳过本帧")
                continue

            if should_update_slam:
                self.first_run = False

                # 检查雷达数据点
                if len(frame.points) == 0:
                    print("⚠️  收到空雷达数据，跳过SLAM更新")
                    continue

                print(f"🔄 准备SLAM更新: 收到 {len(frame.points)} 个雷达点")

                # 处理SLAM更新
                start_time = time.time()
                scan_data, motion_increment, self.last_encoder_state = to_slam_inputs(
                    self.last_encoder_state, frame, self.wheels,
                    max_range_mm=self.max_range, quality_min=0
                )

                # 检查扫描数据有效性
                valid_points = sum(1 for d in scan_data if 0 < d < self.max_range)
                print(f"📊 有效雷达点: {valid_points}/360")

                if valid_points < 10:
                    print("⚠️  有效雷达点太少，跳过SLAM更新")
                    continue

                self.slam.update(scan_data, motion_increment)
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
                print(f"✓ SLAM已更新: {elapsed:.3f}秒 | 位姿: ({current_position.x_meters:.3f}, "
                      f"{current_position.y_meters:.3f}, {current_position.heading_degrees:.1f}°)")
                print(f"  移动增量: dx={motion_increment[0]:.1f}mm, dθ={motion_increment[1]:.2f}°")

                # 记录数据到文件
                if self.logger:
                    # 计算雷达数据统计
                    valid_scans = [dist for dist in scan_data if 0 < dist < self.max_range]
                    min_dist = min(valid_scans) if valid_scans else 0
                    max_dist = max(valid_scans) if valid_scans else 0

                    # 获取任务状态
                    mission_status = None
                    if self.status_report.get("report"):
                        report = self.status_report["report"]
                        phase_name = {
                            Phase.EXPLORING: "探索中",
                            Phase.RETURNING: "返回中",
                            Phase.NAVIGATING_TO_GOAL: "前往目标",
                            Phase.COMPLETED: "已完成"
                        }.get(report.current_phase, report.current_phase.name)

                        mission_status = {
                            'phase': phase_name,
                            'frontiers': report.detected_frontiers,
                            'message': report.status_message
                        }

                    frame_info = {
                        'pos_x': current_position.x_meters,
                        'pos_y': current_position.y_meters,
                        'heading': current_position.heading_degrees,
                        'encoder_l': frame.encoder_l,
                        'encoder_r': frame.encoder_r,
                        'yaw': frame.current_yaw,
                        'cmd_id': frame.cmd_id,
                        'status_text': self._get_status_text(frame.status),
                        'has_movement': self.last_encoder_state is not None,
                        'distance_inc': motion_increment[0],
                        'angle_inc': motion_increment[1],
                        'time_inc': motion_increment[2],
                        'radar_points': len(scan_data),
                        'valid_points': len(valid_scans),
                        'min_dist': min_dist,
                        'max_dist': max_dist,
                        'scan_samples': self._parse_scan_samples(scan_data, sample_interval=30),
                        'mission_status': mission_status
                    }

                    self.logger.log_frame_data(frame_info)

            # ========== 命令重发逻辑 ==========
            # 检查第一条命令是否丢失
            if self.cmd_counter == 1 and frame.cmd_id == 0 and self.cmd_active:
                self.retry_counter += 1

                # 每10帧检查一次
                if self.retry_counter >= self.retry_interval:
                    retry_times = self.retry_counter // self.retry_interval

                    if retry_times <= self.max_retries:
                        print(f"⚠️  小车未响应命令，重发第 {retry_times} 次...")
                        # 重新发送第一条命令
                        self.serial.send_cmd(
                            self.cmd_counter,
                            turn_rad=self.planned_turn,
                            distance_m=0
                        )
                        print(f"→ 重新发送转向命令: {self.planned_turn:.2f}° (命令ID={self.cmd_counter})")
                    else:
                        print("❌ 命令重发次数过多，跳过此命令")
                        self.cmd_active = False
                        self.retry_counter = 0
            # ================================

            # 机器人空闲时发送下一条命令
            if frame.status == 0 and frame.cmd_id == self.cmd_counter:
                if self.cmd_active:
                    self.cmd_active = False
                    self.retry_counter = 0
                    print("✓ 命令执行完成")

                if not self.cmd_active:
                    # 从任务控制器获取新命令
                    if self.step_counter == 0:
                        action = self.mission.compute_next_action(current_position)
                        self.planned_turn, self.planned_distance, self.status_report["report"] = action

                    self.last_robot_status = 1
                    self.cmd_counter += 1

                    # 执行两步策略
                    if self.step_counter == 0:
                        # 步骤1: 只转向
                        self.serial.send_cmd(
                            self.cmd_counter,
                            turn_rad=self.planned_turn,
                            distance_m=0
                        )
                        print(f"→ 发送转向命令: {self.planned_turn:.2f}° (命令ID={self.cmd_counter})")
                        self.step_counter += 1
                    elif self.step_counter == 1:
                        # 步骤2: 只直行
                        self.serial.send_cmd(
                            self.cmd_counter,
                            turn_rad=0,
                            distance_m=self.planned_distance
                        )
                        print(f"→ 发送直行命令: {self.planned_distance:.3f}m (命令ID={self.cmd_counter})")
                        self.step_counter = 0

                    self.cmd_active = True


def main():
    parser = argparse.ArgumentParser(description="自主机器人导航系统(修复版)")
    parser.add_argument("--port", type=str, default="COM7", help="串口名称")
    parser.add_argument("--baud", type=int, default=921600, help="波特率")

    # 【修复BUG-2】统一轮子参数配置
    parser.add_argument("--wheel-radius", type=float, default=0.0325,
                       help="轮子半径(米，默认 0.0325)")
    parser.add_argument("--half-wheelbase", type=float, default=0.084,
                       help="半轴距(米，默认 0.084)")
    parser.add_argument("--cpr", type=int, default=1650,
                       help="编码器每转计数(默认 1560)")

    parser.add_argument("--log-file", type=str, default=None, help="日志文件名")
    parser.add_argument("--no-log", action="store_true", help="禁用数据记录")
    args = parser.parse_args()

    print("=" * 100)
    print("🔧 使用修复版本 - 主要修复:")
    print("  1. ✅ 修复SLAM每帧更新bug")
    print("  2. ✅ 统一轮子参数配置")
    print("  3. ✅ 添加编码器合理性检查")
    print("=" * 100)

    # 初始化数据记录器
    logger = None if args.no_log else DataLogger(args.log_file)
    if logger:
        print(f"📝 数据记录到: {logger.file_path}")

    # 初始化通信 - 使用统一参数
    wheel_geom = BTWheels(args.wheel_radius, args.half_wheelbase, args.cpr)
    serial_link = BTLink(port=args.port, baud=args.baud, logger=logging.getLogger("BTLink"))
    serial_link.start()

    # 初始化SLAM
    map_data = bytearray(GRID_SIZE_PIXELS * GRID_SIZE_PIXELS)
    laser_model = Laser(360, 5.0, 360.0, 8000, 0, 0)
    slam_engine = RMHC_SLAM(
        laser_model, GRID_SIZE_PIXELS, WORLD_SIZE_METERS,
        hole_width_mm=100
    )

    # 初始化地图
    occupancy_grid = OccupancyGrid(GRID_SIZE_PIXELS, WORLD_SIZE_METERS)
    position_state = [0, 0, 0]

    # 【修复BUG-2】初始化任务控制器 - 使用相同参数
    robot_wheels = WheelConfig(
        radius_mm=args.wheel_radius * 1000,  # 转换为mm
        half_baseline_mm=args.half_wheelbase * 1000,  # 转换为mm
        encoder_ticks_per_revolution=args.cpr
    )
    target_coords = (2.3, 2.3)  # 目标位置(米)
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

    # 启动控制线程
    navigation_thread = NavigationThread(
        serial_link, wheel_geom, slam_engine, occupancy_grid,
        controller, position_state, map_data,
        max_radar_range=8000, status_container=status_container,
        data_logger=logger
    )
    navigation_thread.start()

    # 可视化循环
    print("=" * 100)
    print("自主导航系统已启动 (修复版)")
    print("=" * 100)

    try:
        while True:
            status = status_container.get("report", None)

            # 获取机器人位姿
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

                hud_text = (
                    f"阶段: {phase_name}\n"
                    f"边界点数量: {status.detected_frontiers}\n"
                    f"已知区域: {status.tracked_region}\n"
                    f"路径长度: {status.path_length} 节点\n"
                    f"目标已发现: {'是' if status.goal_discovered else '否'}\n"
                    f"状态: {status.status_message}"
                )

            # 渲染可视化
            frontier_list = status.frontier_list if status is not None else None
            best_frontier = status.selected_frontier if status is not None else None
            sample_points = status.sampled_frontiers

            occupancy_grid.renderer.display(
                occupancy_grid.data, robot_pose, extra_text=hud_text,
                roi_rect=(status.tracked_region if status is not None else None),
                frontier_points=frontier_list,
                best_frontier_point=best_frontier,
                spatially_sampled=sample_points,
                exit_pose_pix=status.goal_position
            )

    except KeyboardInterrupt:
        print("\n⚠️  正在关闭系统...")
        if logger:
            logger.log_summary()
            print(f"✓ 数据已保存到: {logger.file_path}")

    finally:
        navigation_thread.stop()
        serial_link.stop()


if __name__ == "__main__":
    main()
