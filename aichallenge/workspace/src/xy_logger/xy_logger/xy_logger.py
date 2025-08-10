#!/usr/bin/env python3


import rclpy, atexit, signal, os, csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from datetime import datetime

# 保存先ディレクトリ
OUT_DIR = "/output"
# タイムスタンプ（例: 2025-08-10_162530）
timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
# ファイル名をタイムスタンプ付きで生成
OUT = os.path.join(OUT_DIR, f"xy_log_{timestamp}.csv")

MAX_DATA_ROWS = 150  # ヘッダ行を除いた最大書き込み行数（データ行数）

class XYLogger(Node):
    def __init__(self):
        super().__init__('xy_logger')

        # 受信: 最新だけ保持
        self.last_msg = None
        self.rows_written = 0
        self.closed = False

        self.sub = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.cb, 10
        )

        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        self.f = open(OUT, 'a', newline='', buffering=1)  # 行バッファリング
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow(['stamp_sec','stamp_nsec','x','y','z','frame_id','child_frame_id'])

        # 1秒周期で最新データを1行だけ書く
        self.timer = self.create_timer(1.0, self.tick)

        # どんな終わり方でも close する
        atexit.register(self._close)
        rclpy.get_default_context().on_shutdown(self._close)
        signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())

    def cb(self, msg: Odometry):
        # 最新のメッセージだけ保持（書き込みはtickで1Hz）
        self.last_msg = msg

    def tick(self):
        # 上限に達したら終了
        if self.rows_written >= MAX_DATA_ROWS:
            self.get_logger().info(f"Reached {MAX_DATA_ROWS} rows. Stopping.")
            self._close()
            # ノードを停止
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        # まだデータ未受信ならスキップ
        if self.last_msg is None:
            return

        msg = self.last_msg
        p = msg.pose.pose.position
        try:
            self.w.writerow([
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.3f}',
                msg.header.frame_id, msg.child_frame_id
            ])
            self.f.flush()  # 逐次フラッシュ（安全重視）
            self.rows_written += 1
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")
            self._close()
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if hasattr(self, 'timer') and self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass
        try:
            if hasattr(self, 'f') and not self.f.closed:
                self.f.flush()
                self.f.close()
        except Exception:
            pass

def main():
    rclpy.init()
    n = XYLogger()
    try:
        rclpy.spin(n)
    finally:
        n._close()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
