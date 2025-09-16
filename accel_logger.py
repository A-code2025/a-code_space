
#!/usr/bin/env python3
import rclpy, atexit, signal, os, csv, math, time
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as RosTime

# Optional message imports (guarded; not all may exist in every env)
try:
    from geometry_msgs.msg import AccelStamped
except Exception:
    AccelStamped = None
try:
    from geometry_msgs.msg import AccelWithCovarianceStamped
except Exception:
    AccelWithCovarianceStamped = None
try:
    from nav_msgs.msg import Odometry
except Exception:
    Odometry = None
# Autoware control variants (any subset may exist)
try:
    from autoware_auto_control_msgs.msg import AckermannControlCommand
except Exception:
    AckermannControlCommand = None
try:
    from autoware_auto_control_msgs.msg import AckermannControlCommandStamped
except Exception:
    AckermannControlCommandStamped = None
try:
    from autoware_auto_control_msgs.msg import LongitudinalCommand
except Exception:
    LongitudinalCommand = None
try:
    from autoware_auto_control_msgs.msg import ControlCommand
except Exception:
    ControlCommand = None
try:
    from autoware_auto_control_msgs.msg import ControlCommandStamped
except Exception:
    ControlCommandStamped = None

DEFAULT_OUT_DIR = "/output"
DEFAULT_TOPIC_CMD = "/control/command/control_cmd"
DEFAULT_TOPIC_ACC = "/localization/acceleration"
DEFAULT_TOPIC_ODO = "/localization/kinematic_state"

class AccelLogNode(Node):
    def __init__(self):
        super().__init__("accel_log_node")

        # Parameters
        self.declare_parameter("out_dir", DEFAULT_OUT_DIR)
        self.declare_parameter("log_hz", 20.0)
        self.declare_parameter("max_rows", 20000)
        self.declare_parameter("topic_control_cmd", DEFAULT_TOPIC_CMD)
        self.declare_parameter("topic_accel", DEFAULT_TOPIC_ACC)
        self.declare_parameter("topic_odom", DEFAULT_TOPIC_ODO)

        self.out_dir   = self.get_parameter("out_dir").get_parameter_value().string_value
        self.log_hz    = float(self.get_parameter("log_hz").get_parameter_value().double_value)
        self.max_rows  = int(self.get_parameter("max_rows").get_parameter_value().integer_value)
        self.topic_cmd = self.get_parameter("topic_control_cmd").get_parameter_value().string_value
        self.topic_acc = self.get_parameter("topic_accel").get_parameter_value().string_value
        self.topic_odo = self.get_parameter("topic_odom").get_parameter_value().string_value

        # State
        self.rows_written = 0
        self.closed = False
        self.last_target_acc = None      # (a_des, stamp_ros)
        self.last_meas_acc   = None      # (a_meas, stamp_ros, source)
        self.last_speed      = None      # (v_mps, stamp_ros)
        self.prev_speed_for_deriv = None # (v_mps, stamp_ros)

        # Output CSV
        os.makedirs(self.out_dir, exist_ok=True)
        timestamp = time.strftime("%Y-%m-%d_%H%M%S", time.localtime())
        self.out_path = os.path.join(self.out_dir, f"accel_log_{timestamp}.csv")
        self.f = open(self.out_path, "w", newline="", buffering=1)
        self.w = csv.writer(self.f)
        self.w.writerow(["stamp_sec", "v_mps", "a_target_mps2", "a_current_mps2", "a_current_source"])
        self.get_logger().info(f"Writing: {self.out_path}")

        # Subscriptions: control command (try multiple message types to maximize compatibility)
        self._subs = []
        def _sub(msg_type):
            if msg_type is None: 
                return
            try:
                s = self.create_subscription(msg_type, self.topic_cmd, self.cb_control, 20)
                self._subs.append(s)
                self.get_logger().info(f"Subscribed to {self.topic_cmd} as {msg_type.__name__}")
            except Exception as e:
                self.get_logger().warn(f"Failed to subscribe {msg_type}: {e}")

        _sub(AckermannControlCommandStamped)
        _sub(ControlCommandStamped)
        _sub(AckermannControlCommand)
        _sub(ControlCommand)
        _sub(LongitudinalCommand)

        # Acceleration measurement (prefer direct accel topic)
        if AccelWithCovarianceStamped is not None:
            try:
                self._subs.append(self.create_subscription(AccelWithCovarianceStamped, self.topic_acc, self.cb_accel_cov, 20))
                self.get_logger().info(f"Subscribed to {self.topic_acc} (AccelWithCovarianceStamped)")
            except Exception as e:
                self.get_logger().warn(f"AccelWithCovarianceStamped sub failed: {e}")
        if AccelStamped is not None:
            try:
                self._subs.append(self.create_subscription(AccelStamped, self.topic_acc, self.cb_accel, 20))
                self.get_logger().info(f"Subscribed to {self.topic_acc} (AccelStamped)")
            except Exception as e:
                self.get_logger().warn(f"AccelStamped sub failed: {e}")

        # Odometry for speed & fallback acceleration (derivative)
        if Odometry is not None:
            try:
                self._subs.append(self.create_subscription(Odometry, self.topic_odo, self.cb_odom, 50))
                self.get_logger().info(f"Subscribed to {self.topic_odo} (Odometry)")
            except Exception as e:
                self.get_logger().warn(f"Odometry sub failed: {e}")

        # Timer
        period = 1.0/max(self.log_hz, 1e-6)
        self.timer = self.create_timer(period, self.tick)

        # Cleanup hooks
        atexit.register(self._close)
        rclpy.get_default_context().on_shutdown(self._close)
        signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())

    # ---------- Callbacks ----------
    def cb_control(self, msg):
        a = self._extract_target_accel(msg)
        if a is not None:
            stamp = self._extract_stamp(msg)
            self.last_target_acc = (a, stamp)

    def cb_accel_cov(self, msg):
        try:
            a = float(msg.accel.accel.linear.x)
        except Exception:
            return
        stamp = msg.header.stamp
        self.last_meas_acc = (a, stamp, "localization/accel_cov")

    def cb_accel(self, msg):
        try:
            a = float(msg.accel.linear.x)
        except Exception:
            return
        stamp = msg.header.stamp
        self.last_meas_acc = (a, stamp, "localization/accel")

    def cb_odom(self, msg):
        try:
            v = float(msg.twist.twist.linear.x)
        except Exception:
            v = None
        stamp = msg.header.stamp if hasattr(msg, "header") else None
        if v is not None:
            self.last_speed = (v, stamp)
            # derive accel if we have previous
            if self.prev_speed_for_deriv is not None:
                v0, t0 = self.prev_speed_for_deriv
                dt = self._dt_sec(t0, stamp)
                if dt > 1e-3:
                    a_est = (v - v0) / dt
                    self.last_meas_acc = (a_est, stamp, "from_odometry_dvdt")
            self.prev_speed_for_deriv = (v, stamp)

    # ---------- Timer tick ----------
    def tick(self):
        if self.rows_written >= self.max_rows:
            self.get_logger().info(f"Reached {self.max_rows} rows. Stopping.")
            self._close()
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        # Compose one row
        stamp = self._now_ros_time()
        v = self.last_speed[0] if self.last_speed is not None else float("nan")
        a_tar = self.last_target_acc[0] if self.last_target_acc is not None else float("nan")
        if self.last_meas_acc is not None:
            a_meas, stamp_meas, src = self.last_meas_acc
            src_name = src
        else:
            a_meas = float("nan")
            src_name = ""

        # Write
        try:
            self.w.writerow([f"{self._time_to_sec(stamp):.6f}", f"{v:.6f}", f"{a_tar:.6f}", f"{a_meas:.6f}", src_name])
            self.f.flush()
            self.rows_written += 1
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")
            self._close()
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ---------- Utilities ----------
    def _extract_target_accel(self, msg):
        # Try common Autoware structures
        try:
            # Stamped → inner
            inner = getattr(msg, "control", msg)  # ControlCommandStamped.control or same object
            # Direct longitudinal
            if hasattr(inner, "longitudinal"):
                lon = inner.longitudinal
                if hasattr(lon, "acceleration_mps2"):
                    return float(lon.acceleration_mps2)
                if hasattr(lon, "acceleration"):
                    return float(lon.acceleration)
            # Ackermann-like: top-level fields
            for name in ("acceleration_mps2", "acceleration"):
                if hasattr(inner, name):
                    return float(getattr(inner, name))
        except Exception:
            pass
        # Fallback: try one more level (e.g., msg.command.longitudinal.*)
        try:
            cmd = getattr(msg, "command", None)
            if cmd is not None and hasattr(cmd, "longitudinal"):
                lon = cmd.longitudinal
                for name in ("acceleration_mps2", "acceleration"):
                    if hasattr(lon, name):
                        return float(getattr(lon, name))
        except Exception:
            pass
        return None

    def _extract_stamp(self, msg):
        # Try typical header nesting
        for attr in ("header", "control", "command"):
            obj = getattr(msg, attr, None)
            if obj is None:
                continue
            if hasattr(obj, "stamp"):
                st = getattr(obj, "stamp")
                if isinstance(st, RosTime):
                    return st
        # No header: return now
        return self._now_ros_time()

    def _now_ros_time(self):
        return self.get_clock().now().to_msg()

    @staticmethod
    def _dt_sec(t0, t1):
        if t0 is None or t1 is None:
            return 0.0
        return (t1.sec + t1.nanosec*1e-9) - (t0.sec + t0.nanosec*1e-9)

    @staticmethod
    def _time_to_sec(t):
        return t.sec + t.nanosec*1e-9

    def _close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if hasattr(self, "timer") and self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass
        try:
            if hasattr(self, "f") and not self.f.closed:
                self.f.flush()
                self.f.close()
                self.get_logger().info(f"Closed {self.out_path}")
        except Exception:
            pass

def main():
    rclpy.init()
    node = AccelLogNode()
    try:
        rclpy.spin(node)
    finally:
        node._close()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
