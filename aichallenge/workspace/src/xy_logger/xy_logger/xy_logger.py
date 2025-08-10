#!/usr/bin/env python3
import rclpy, atexit, signal, os, csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from datetime import datetime
import os

# 保存先ディレクトリ
OUT_DIR = "/output"

# タイムスタンプ（例: 2025-08-10_162530）
timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")

# ファイル名をタイムスタンプ付きで生成
OUT = os.path.join(OUT_DIR, f"xy_log_{timestamp}.csv")

class XYLogger(Node):
    def __init__(self):
        super().__init__('xy_logger')
        self.sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.cb, 10)
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        self.f = open(OUT, 'a', newline='', buffering=1)  # 行バッファリング
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow(['stamp_sec','stamp_nsec','x','y','z','frame_id','child_frame_id'])
        # どんな終わり方でも close する
        atexit.register(self._close)
        rclpy.get_default_context().on_shutdown(self._close)
        signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())

    def cb(self, msg):
        p = msg.pose.pose.position
        self.w.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec,
                         f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.3f}',
                         msg.header.frame_id, msg.child_frame_id])
        self.f.flush()  # 逐次フラッシュ（安全重視）

    def _close(self):
        try:
            self.f.flush(); self.f.close()
        except Exception:
            pass

def main():
    rclpy.init()
    n = XYLogger()
    try:
        rclpy.spin(n)
    finally:
        n._close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

