#!/usr/bin/env python3
import rclpy, atexit, signal, os, csv, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from datetime import datetime

OUT_DIR = "/output"
timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
OUT = os.path.join(OUT_DIR, f"xy_log_{timestamp}.csv")

# 添付のラインCSV（必要に応じてパラメータで差し替え可）
DEFAULT_RACELINE_CSV = "/aichallenge/workspace/src/aichallenge_submit/simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv"

MAX_DATA_ROWS = 1000  # データ行上限（ヘッダ除く）

class XYLogger(Node):
    def __init__(self):
        super().__init__('xy_logger')

        self.last_msg = None
        self.rows_written = 0
        self.closed = False

        self.sub = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.cb, 20
        )

        # raceline CSV をロード
        self.raceline_csv = self.declare_parameter('raceline_csv', DEFAULT_RACELINE_CSV).value
        self.raceline = self._load_raceline(self.raceline_csv)  # list[dict] or None

        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        self.f = open(OUT, 'a', newline='', buffering=1)
        self.w = csv.writer(self.f)

        # ヘッダは7列のみ
        if self.f.tell() == 0:
            self.w.writerow(['x','y','z','x_quat','y_quat','z_quat','w_quat'])

        # 0.5秒ごとに書き込み
        self.timer = self.create_timer(3.0, self.tick)

        # 終了時クリーンアップ
        atexit.register(self._close)
        rclpy.get_default_context().on_shutdown(self._close)
        signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())

    def cb(self, msg: Odometry):
        self.last_msg = msg

    def tick(self):
        if self.rows_written >= MAX_DATA_ROWS:
            self.get_logger().info(f"Reached {MAX_DATA_ROWS} rows. Stopping.")
            self._close()
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        if self.last_msg is None:
            return

        msg = self.last_msg
        p = msg.pose.pose.position
        # 7列のベース: x, y, z は Odometry の位置
        row = [f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}']

        # quat は raceline があれば最近傍の行から、無ければ Odometry の姿勢を使用
        if self.raceline is not None:
            _, rl = self._nearest_on_raceline(p.x, p.y)
            row += [
                f'{rl["x_quat"]:.6f}', f'{rl["y_quat"]:.6f}',
                f'{rl["z_quat"]:.6f}', f'{rl["w_quat"]:.6f}'
            ]
        else:
            q = msg.pose.pose.orientation
            row += [f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}']

        try:
            self.w.writerow(row)
            self.f.flush()
            self.rows_written += 1
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")
            self._close()
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _load_raceline(self, path):
        try:
            if not os.path.exists(path):
                self.get_logger().warn(f"raceline CSV not found: {path}")
                return None
            data = []
            with open(path, newline='') as f:
                r = csv.DictReader(f)
                for i, row in enumerate(r):
                    try:
                        data.append({
                            "x": float(row.get("x", 0.0)),
                            "y": float(row.get("y", 0.0)),
                            "z": float(row.get("z", 0.0)),
                            "x_quat": float(row.get("x_quat", 0.0)),
                            "y_quat": float(row.get("y_quat", 0.0)),
                            "z_quat": float(row.get("z_quat", 0.0)),
                            "w_quat": float(row.get("w_quat", 1.0)),
                        })
                    except Exception as ex:
                        self.get_logger().warn(f"Skip bad row {i}: {ex}")
            if not data:
                self.get_logger().warn(f"raceline CSV empty: {path}")
                return None
            self.get_logger().info(f"Loaded raceline: {len(data)} points from {path}")
            return data
        except Exception as e:
            self.get_logger().error(f"Failed to load raceline CSV: {e}")
            return None

    def _nearest_on_raceline(self, x, y):
        best_i = 0
        best_d2 = float('inf')
        rl_best = None
        for i, rl in enumerate(self.raceline):
            dx = rl["x"] - x
            dy = rl["y"] - y
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                rl_best = rl
        return best_i, rl_best

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

