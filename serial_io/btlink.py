# -*- coding: utf-8 -*-
"""
btlink_improved.py - 改进版蓝牙通信模块
支持编码器帧（10ms）和雷达帧（200ms）分离传输

新增功能：
1. 双帧格式解析（编码器帧 0xFD 0xDF + 雷达帧 0x55 0xAA）
2. 独立队列管理（编码器队列maxsize=100 + 雷达队列maxsize=1）
3. 编码器累积接口

上行数据帧：
  编码器帧(10ms): 0xFD 0xDF | time_us | cmd_id | status | encoder_l | encoder_r | yaw
  雷达帧(200ms): 0x55 0xAA | data_count | [quality, angle_q6, distance_q2] × N

下行命令帧（不变）：
  0xAA 0x55 | len=10 | cmd_id | turn_rad | distance_m | CRC16
"""

from __future__ import annotations
import struct
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Tuple
from queue import Queue, Full, Empty

import serial
import serial.tools.list_ports

# ================= 常量 =================

# 帧头常量
HEADER_DOWN = b'\xAA\x55'  # PC → STM32
HEADER_ENCODER = b'\xFD\xDF'  # STM32 → PC (编码器)
HEADER_LIDAR = b'\x55\xAA'  # STM32 → PC (雷达)

DEFAULT_BAUD = 921600
DEFAULT_TIMEOUT = 0.1
RECONNECT_INTERVAL = 2.0


def crc16_modbus(data: bytes) -> int:
    """CRC16(Modbus) - init 0xFFFF, poly 0xA001"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 1) != 0:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ================= 数据结构 =================

@dataclass
class WheelGeom:
    """轮子几何参数"""
    wheel_radius_m: float  # 轮半径(米)
    half_wheelbase_m: float  # 半轮距(米)
    ticks_per_rev: int  # 编码器CPR


@dataclass
class LidarPoint:
    """激光雷达点"""
    quality: int
    angle_deg: float
    distance_mm: float


@dataclass
class EncoderFrame:
    """编码器数据帧（轻量级，10ms）"""
    time_us: int
    cmd_id: int
    status: int  # 0=完成, 1=转向, 2=直行
    encoder_l: int  # 左轮累计值
    encoder_r: int  # 右轮累计值
    current_yaw: float  # 航向角（保留）


@dataclass
class LidarFrame:
    """雷达数据帧（重量级，200ms）"""
    data_count: int
    points: List[LidarPoint]


# ================= BTLink 主类 =================

class BTLink:
    """
    改进版蓝牙串口通信管理

    支持双帧格式：
    - 编码器帧（10ms）: 21字节，高频率
    - 雷达帧（200ms）: 1804字节，低频率

    用法：
        link = BTLink('COM7', 921600)
        link.start()

        # 获取编码器帧
        enc = link.get_encoder_frame(timeout=0.05)

        # 获取雷达帧
        lidar = link.get_lidar_frame(timeout=1.0)

        # 发送命令
        link.send_cmd(cmd_id=1, turn_rad=0.5, distance_m=0.0)

        link.stop()
    """

    def __init__(self,
                 port: Optional[str],
                 baud: int = DEFAULT_BAUD,
                 timeout: float = DEFAULT_TIMEOUT,
                 auto_reconnect: bool = True,
                 encoder_queue_size: int = 100,  # 编码器队列：存储1秒数据
                 lidar_queue_size: int = 1,  # 雷达队列：只保留最新
                 logger: Optional[logging.Logger] = None):

        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect
        self.log = logger or logging.getLogger("BTLink")

        self._ser: Optional[serial.Serial] = None
        self._stop_evt = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_buf = bytearray()

        # 双队列系统
        self._encoder_queue: Queue[EncoderFrame] = Queue(maxsize=encoder_queue_size)
        self._lidar_queue: Queue[LidarFrame] = Queue(maxsize=lidar_queue_size)

        # 统计信息
        self.stats = {
            'encoder_frames': 0,
            'lidar_frames': 0,
            'encoder_dropped': 0,
            'lidar_dropped': 0
        }

    # ========== 生命周期 ==========

    def start(self):
        """启动通信"""
        self._stop_evt.clear()
        self._open_serial()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="BTLink-RX", daemon=True)
        self._rx_thread.start()
        self.log.info("BTLink started (improved version)")

    def stop(self):
        """停止通信"""
        self._stop_evt.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.5)
        self._close_serial()
        self.log.info("BTLink stopped")

    # ========== 下行发送（PC → STM32）==========

    def send_cmd(self, cmd_id: int, turn_rad: float, distance_m: float):
        """
        发送运动命令
        格式: AA 55 | len=10 | cmd_id(u16) | turn_rad(f32) | distance_m(f32) | CRC16
        """
        payload = struct.pack('<Hff', int(cmd_id), float(turn_rad), float(distance_m))
        body_wo_crc = HEADER_DOWN + struct.pack('<H', 10) + payload
        crc = struct.pack('<H', crc16_modbus(body_wo_crc))
        frame = body_wo_crc + crc
        self._write(frame)
        self.log.debug(f"send_cmd: id={cmd_id} turn={turn_rad:.4f}rad dist={distance_m:.4f}m")

    # ========== 上行获取（STM32 → PC）==========

    def get_encoder_frame(self, timeout: Optional[float] = None) -> Optional[EncoderFrame]:
        """获取编码器帧（FIFO）"""
        try:
            return self._encoder_queue.get(timeout=timeout)
        except Empty:
            return None

    def get_lidar_frame(self, timeout: Optional[float] = None) -> Optional[LidarFrame]:
        """获取雷达帧（最新）"""
        try:
            return self._lidar_queue.get(timeout=timeout)
        except Empty:
            return None

    def get_latest_lidar_frame(self) -> Optional[LidarFrame]:
        """只获取最新的雷达帧（清空队列）"""
        try:
            frm = self._lidar_queue.get(timeout=0.0)
        except Empty:
            return None

        # 清空剩余，保留最新
        while True:
            try:
                frm = self._lidar_queue.get_nowait()
            except Empty:
                break
        return frm

    def get_all_pending_encoders(self) -> List[EncoderFrame]:
        """获取队列中所有待处理的编码器帧"""
        frames = []
        while True:
            try:
                frames.append(self._encoder_queue.get_nowait())
            except Empty:
                break
        return frames

    # ========== 私有：串口操作 ==========

    def _open_serial(self):
        """打开串口（带重连）"""
        while not self._stop_evt.is_set():
            try:
                if self._ser and self._ser.is_open:
                    return
                if self.port is None:
                    self.port = self._auto_pick_port()
                self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                self.log.info(f"Serial opened: {self.port} @ {self.baud}")
                return
            except Exception as e:
                self.log.warning(f"Open serial failed: {e}")
                if not self.auto_reconnect:
                    raise
                time.sleep(RECONNECT_INTERVAL)

    def _close_serial(self):
        """关闭串口"""
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        finally:
            self._ser = None

    def _write(self, data: bytes):
        """写入数据到串口"""
        if not self._ser or not self._ser.is_open:
            self._open_serial()
        try:
            self._ser.write(data)
            self._ser.flush()
        except Exception as e:
            self.log.error(f"Serial write failed: {e}")
            if self.auto_reconnect:
                self._close_serial()

    # ========== RX 接收线程 ==========

    def _rx_loop(self):
        """接收线程主循环"""
        buf = self._rx_buf

        while not self._stop_evt.is_set():
            # 确保串口打开
            if not self._ser or not self._ser.is_open:
                if not self.auto_reconnect:
                    time.sleep(0.1)
                    continue
                self._open_serial()
                time.sleep(0.1)
                continue

            try:
                chunk = self._ser.read(512)
                if chunk:
                    buf += chunk

                    # 循环解析所有完整帧
                    while True:
                        # 尝试解析编码器帧
                        enc_frame = self._try_parse_encoder_frame(buf)
                        if enc_frame:
                            self._enqueue_encoder(enc_frame)
                            continue

                        # 尝试解析雷达帧
                        lidar_frame = self._try_parse_lidar_frame(buf)
                        if lidar_frame:
                            self._enqueue_lidar(lidar_frame)
                            continue

                        # 没有完整帧，等待更多数据
                        break

            except Exception as e:
                self.log.warning(f"Serial read error: {e}")
                if self.auto_reconnect:
                    self._close_serial()
                    time.sleep(RECONNECT_INTERVAL)
                else:
                    break

    # ========== 帧解析 ==========

    def _try_parse_encoder_frame(self, buf: bytearray) -> Optional[EncoderFrame]:
        """
        解析编码器帧
        格式: FD DF | time_us(4) | cmd_id(2) | status(1) | encoder_l(4) | encoder_r(4) | yaw(4)
        总长: 21字节
        """
        # 查找帧头
        start = buf.find(HEADER_ENCODER)
        if start == -1:
            return None

        # 移除无用前缀
        if start > 0:
            del buf[:start]

        # 检查长度
        if len(buf) < 21:
            return None

        try:
            time_us, = struct.unpack_from('<I', buf, 2)
            cmd_id, = struct.unpack_from('<H', buf, 6)
            status = buf[8]
            encoder_l, = struct.unpack_from('<i', buf, 9)
            encoder_r, = struct.unpack_from('<i', buf, 13)
            current_yaw, = struct.unpack_from('<f', buf, 17)
        except struct.error:
            return None

        # 移除已解析的帧
        del buf[:21]

        return EncoderFrame(
            time_us=time_us,
            cmd_id=cmd_id,
            status=status,
            encoder_l=encoder_l,
            encoder_r=encoder_r,
            current_yaw=current_yaw
        )

    def _try_parse_lidar_frame(self, buf: bytearray) -> Optional[LidarFrame]:
        """
        解析雷达帧
        格式: 55 AA | data_count(2) | [quality(1) + angle_q6(2) + distance_q2(2)] × N
        """
        # 查找帧头
        start = buf.find(HEADER_LIDAR)
        if start == -1:
            return None

        # 移除无用前缀
        if start > 0:
            del buf[:start]

        # 至少需要帧头+data_count
        if len(buf) < 4:
            return None

        try:
            data_count, = struct.unpack_from('<H', buf, 2)
        except struct.error:
            return None

        # 合理性检查
        if data_count > 2048:
            del buf[0]  # 丢弃一个字节，继续搜索
            return None

        total_len = 4 + data_count * 5
        if len(buf) < total_len:
            return None  # 数据未收齐

        # 解析点云
        points: List[LidarPoint] = []
        off = 4
        for _ in range(data_count):
            q = buf[off]
            angle_q6, = struct.unpack_from('<H', buf, off + 1)
            dist_q2, = struct.unpack_from('<H', buf, off + 3)
            off += 5

            angle_deg = angle_q6 / 64.0
            distance_mm = dist_q2 / 4.0
            points.append(LidarPoint(q, angle_deg, distance_mm))

        # 移除已解析的帧
        del buf[:total_len]

        return LidarFrame(data_count=data_count, points=points)

    # ========== 入队管理 ==========

    def _enqueue_encoder(self, frame: EncoderFrame):
        """编码器帧入队"""
        try:
            self._encoder_queue.put_nowait(frame)
            self.stats['encoder_frames'] += 1
        except Full:
            # 队列满，丢弃最旧的
            try:
                _ = self._encoder_queue.get_nowait()
                self.stats['encoder_dropped'] += 1
            except Empty:
                pass
            self._encoder_queue.put_nowait(frame)

    def _enqueue_lidar(self, frame: LidarFrame):
        """雷达帧入队（丢旧保新）"""
        try:
            self._lidar_queue.put_nowait(frame)
            self.stats['lidar_frames'] += 1
        except Full:
            try:
                _ = self._lidar_queue.get_nowait()
                self.stats['lidar_dropped'] += 1
            except Empty:
                pass
            self._lidar_queue.put_nowait(frame)

    # ========== 工具方法 ==========

    @staticmethod
    def _auto_pick_port() -> Optional[str]:
        """自动选择蓝牙串口"""
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            name = (p.description or '') + ' ' + (p.device or '')
            if 'Bluetooth' in name or 'HC' in name or 'SPP' in name:
                return p.device
        return ports[0].device if ports else None

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            **self.stats,
            'encoder_queue_size': self._encoder_queue.qsize(),
            'lidar_queue_size': self._lidar_queue.qsize()
        }


# ================= 辅助函数：数据转换 =================

def ticks_to_motion(prev: Tuple[int, int, int],
                    curr: Tuple[int, int, int],
                    geom: WheelGeom) -> Tuple[float, float, float]:
    """
    由两个编码器状态计算运动增量
    输入: (cumL, cumR, ts_us)
    输出: (dxy_mm, dtheta_deg, dt_s)
    """
    cumL0, cumR0, ts0 = prev
    cumL1, cumR1, ts1 = curr

    dL = cumL1 - cumL0
    dR = cumR1 - cumR0

    revL = dL / float(geom.ticks_per_rev)
    revR = dR / float(geom.ticks_per_rev)

    sL = (2.0 * 3.141592653589793 * geom.wheel_radius_m) * revL
    sR = (2.0 * 3.141592653589793 * geom.wheel_radius_m) * revR

    ds = 0.5 * (sL + sR)
    dtheta_rad = (sR - sL) / (2.0 * geom.half_wheelbase_m)
    dt_s = (ts1 - ts0) / 1e6

    return (ds * 1000.0, dtheta_rad * 180.0 / 3.141592653589793, dt_s)


def build_scan360(points: List[LidarPoint],
                  max_range_mm: int = 8000,
                  quality_min: int = 0) -> List[int]:
    """
    将雷达点云转换为360度扫描数组（从520个点）
    """
    # 第一步：聚合最小距离
    vals: List[Optional[int]] = [None] * 360
    for p in points:
        if p.quality < quality_min:
            continue
        a = p.angle_deg % 360.0
        idx = int(round(a)) % 360
        d = int(max(0, min(65535, round(p.distance_mm))))
        if d <= 0:
            continue
        if vals[idx] is None or d < vals[idx]:
            vals[idx] = d

    # 全缺失处理
    if all(v is None for v in vals):
        return [max_range_mm] * 360

    # 缺失值填充
    for idx in range(360):
        if vals[idx] is None:
            left_val = None
            right_val = None
            search_range = 30

            for offset in range(1, search_range + 1):
                if left_val is None:
                    left_idx = (idx - offset) % 360
                    if vals[left_idx] is not None:
                        left_val = vals[left_idx]

                if right_val is None:
                    right_idx = (idx + offset) % 360
                    if vals[right_idx] is not None:
                        right_val = vals[right_idx]

                if left_val is not None and right_val is not None:
                    break

            if left_val is not None and right_val is not None:
                vals[idx] = int((left_val + right_val) / 2)
            elif left_val is not None:
                vals[idx] = left_val
            elif right_val is not None:
                vals[idx] = right_val
            else:
                vals[idx] = max_range_mm

    scan = [int(v) if v is not None else max_range_mm for v in vals]
    return reorder_scan_ccw_minus180_to_plus179(scan)


def reorder_scan_ccw_minus180_to_plus179(scan: List[int]) -> List[int]:
    """
    重排序扫描数据
    输入: [0°→359°]
    输出: [-180°→+179°]
    """
    part1 = scan[:181]
    part2 = scan[181:360]
    part1.reverse()
    part2.reverse()
    return part1 + part2
