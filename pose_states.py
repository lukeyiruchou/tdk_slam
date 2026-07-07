#!/usr/bin/env python3
"""
pose_stats.py — Cartographer 定位量化工具

模式 1(預設): 統計 N 秒內 /robot_pose 的 x, y, yaw 平均 / 標準差 / 最大跳動範圍
    ros2 run 前先 source,然後:
    python3 pose_stats.py --duration 60 --csv run1.csv

模式 2(--wait-converge): 從腳本啟動開始計時,偵測 pose 收斂(滑動視窗內
    x/y 標準差 < 門檻,持續 hold 秒),輸出收斂時間。用於重複定位精度 / 最快定位速度測試。
    python3 pose_stats.py --wait-converge --timeout 120

topic 型別自動偵測 (PoseStamped / Pose / PoseWithCovarianceStamped / Odometry)。
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def circ_mean(angles):
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def ang_diff(a, b):
    """回傳 a-b 正規化到 (-pi, pi]"""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d <= -math.pi:
        d += 2 * math.pi
    return d


def mean_std(vals):
    n = len(vals)
    m = sum(vals) / n
    var = sum((v - m) ** 2 for v in vals) / n
    return m, math.sqrt(var)


class PoseStats(Node):
    def __init__(self, args):
        super().__init__('pose_stats')
        self.args = args
        self.samples = []  # (t, x, y, yaw)
        self.start_time = time.monotonic()
        self.converged_at = None
        self.done = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 自動偵測 topic 型別
        msg_type = self._detect_type(args.topic)
        if msg_type is None:
            print(f'[錯誤] 找不到 topic {args.topic},確認 cartographer 已啟動', file=sys.stderr)
            sys.exit(1)
        print(f'[資訊] 訂閱 {args.topic} ({msg_type.__name__})')
        self.create_subscription(msg_type, args.topic, self._cb, qos)
        self.create_timer(0.2, self._check)

    def _detect_type(self, topic):
        table = {
            'geometry_msgs/msg/PoseStamped': PoseStamped,
            'geometry_msgs/msg/Pose': Pose,
            'geometry_msgs/msg/PoseWithCovarianceStamped': PoseWithCovarianceStamped,
            'nav_msgs/msg/Odometry': Odometry,
        }
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            for name, types in self.get_topic_names_and_types():
                if name == topic and types:
                    return table.get(types[0])
            time.sleep(0.2)
        return None

    def _extract(self, msg):
        if isinstance(msg, PoseStamped):
            p = msg.pose
        elif isinstance(msg, PoseWithCovarianceStamped):
            p = msg.pose.pose
        elif isinstance(msg, Odometry):
            p = msg.pose.pose
        else:
            p = msg
        yaw = quat_to_yaw(p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        return p.position.x, p.position.y, yaw

    def _cb(self, msg):
        if self.done:
            return
        x, y, yaw = self._extract(msg)
        self.samples.append((time.monotonic() - self.start_time, x, y, yaw))

    def _check(self):
        elapsed = time.monotonic() - self.start_time
        if self.args.wait_converge:
            if self._is_converged():
                self.converged_at = self.samples[-1][0]
                self._finish()
            elif elapsed > self.args.timeout:
                print(f'\n[結果] 超時 {self.args.timeout:.0f} s 未收斂 → 記為失敗')
                self._finish(report=False)
        else:
            remaining = self.args.duration - elapsed
            print(f'\r收集中… {elapsed:5.1f}/{self.args.duration:.0f} s,'
                  f' {len(self.samples)} 筆', end='', flush=True)
            if remaining <= 0:
                print()
                self._finish()

    def _is_converged(self):
        win, hold = self.args.window, self.args.hold
        thr = self.args.threshold
        if not self.samples:
            return False
        now = self.samples[-1][0]
        # 需要持續 hold 秒都滿足:檢查最近 hold 秒內每個 window 視窗
        recent = [s for s in self.samples if s[0] >= now - win - hold]
        if not recent or recent[0][0] > now - win - hold + 0.5:
            return False  # 資料還不夠長
        # 簡化:取最近 (win+hold) 秒整段的 x/y std 都要 < thr
        xs = [s[1] for s in recent]
        ys = [s[2] for s in recent]
        _, sx = mean_std(xs)
        _, sy = mean_std(ys)
        return sx < thr and sy < thr

    def _finish(self, report=True):
        self.done = True
        if report and self.samples:
            self._report()
        if self.args.csv and self.samples:
            with open(self.args.csv, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'yaw_rad'])
                w.writerows(self.samples)
            print(f'[資訊] 原始資料已存 {self.args.csv}')
        rclpy.shutdown()

    def _report(self):
        xs = [s[1] for s in self.samples]
        ys = [s[2] for s in self.samples]
        yaws = [s[3] for s in self.samples]

        mx, sx = mean_std(xs)
        my, sy = mean_std(ys)
        ym = circ_mean(yaws)
        ydiffs = [ang_diff(a, ym) for a in yaws]
        _, sy_yaw = mean_std(ydiffs)
        yaw_range = max(ydiffs) - min(ydiffs)

        print('=' * 52)
        if self.converged_at is not None:
            print(f'收斂時間: {self.converged_at:.2f} s '
                  f'(門檻 {self.args.threshold*100:.1f} cm, 視窗 {self.args.window:.0f} s)')
            print('-' * 52)
        n = len(self.samples)
        dur = self.samples[-1][0] - self.samples[0][0] if n > 1 else 0
        print(f'樣本數: {n}  ({n/dur:.1f} Hz)' if dur > 0 else f'樣本數: {n}')
        print(f'{"":6}{"平均":>12}{"標準差":>12}{"max-min":>12}')
        print(f'x  (m){mx:12.4f}{sx:12.4f}{max(xs)-min(xs):12.4f}')
        print(f'y  (m){my:12.4f}{sy:12.4f}{max(ys)-min(ys):12.4f}')
        print(f'yaw(°){math.degrees(ym):12.3f}{math.degrees(sy_yaw):12.3f}'
              f'{math.degrees(yaw_range):12.3f}')
        print(f'std cm:  x={sx*100:.2f}  y={sy*100:.2f}   '
              f'range cm: x={(max(xs)-min(xs))*100:.2f}  y={(max(ys)-min(ys))*100:.2f}')
        print('=' * 52)


def main():
    ap = argparse.ArgumentParser(description='Cartographer 定位量化工具')
    ap.add_argument('--topic', default='/robot_pose')
    ap.add_argument('--duration', type=float, default=60.0,
                    help='統計模式收集秒數 (預設 60)')
    ap.add_argument('--csv', default=None, help='原始資料輸出 CSV 路徑')
    ap.add_argument('--wait-converge', action='store_true',
                    help='收斂計時模式:偵測 pose 穩定並輸出耗時')
    ap.add_argument('--threshold', type=float, default=0.01,
                    help='收斂判定 x/y 標準差門檻 (m,預設 0.01)')
    ap.add_argument('--window', type=float, default=2.0,
                    help='收斂判定滑動視窗 (s,預設 2)')
    ap.add_argument('--hold', type=float, default=3.0,
                    help='需持續穩定秒數 (預設 3)')
    ap.add_argument('--timeout', type=float, default=120.0,
                    help='收斂模式最長等待 (s,預設 120)')
    args = ap.parse_args()

    rclpy.init()
    node = PoseStats(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()