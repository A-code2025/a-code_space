#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Trajectory Follower (FF + Stanley + PID)
- Reads a CSV trajectory with columns including s_m, x, y, heading_rad, curvature_1pm, v_ref_mps
- Subscribes to Odometry (/localization/kinematic_state by default)
- Publishes steering (rad) and longitudinal accel command (m/s^2) or pedal (0..1)

Usage (example):
  python3 traj_follower_node.py \
    --ros-args \
      -p traj_csv:=/path/to/trajectory_with_speed.csv \
      -p wheelbase:=1.05 \
      -p steer_topic:=/cmd/steer \
      -p accel_topic:=/cmd/accel

Notes:
- Convert accel command to your vehicle's pedal command in ax_to_pedal() as needed.
- The node assumes a closed loop track; wrap-around is handled via s modulo.
"""

import math
import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi]."""
    a = (angle + math.pi) % (2 * math.pi) - math.pi
    return a


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw from quaternion (ENU/FLU typical)."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Gains:
    ke: float = 1.2          # Stanley lateral gain
    kpsi: float = 1.7        # Yaw error gain
    eps_v: float = 0.1       # Avoid div by zero in Stanley term

@dataclass
class Lookahead:
    l0: float = 0.9         # base lookahead [m]
    tau: float = 0.25        # seconds -> Ld = l0 + tau*v
    Lmin: float = 0.8
    Lmax: float = 6.0

@dataclass
class PID:
    kp: float = 1.0
    ki: float = 0.2
    kd: float = 0.0
    i_min: float = -2.0
    i_max: float = 2.0

    def reset(self):
        self._i = 0.0
        self._e_prev = None

    def step(self, e: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        # integral with anti-windup
        self._i = getattr(self, "_i", 0.0) + e * dt
        self._i = max(self.i_min, min(self.i_max, self._i))
        # derivative
        de = 0.0
        if getattr(self, "_e_prev", None) is not None:
            de = (e - self._e_prev) / dt
        self._e_prev = e
        return self.kp * e + self.ki * self._i + self.kd * de


class Trajectory:
    def __init__(self, csv_path: str):
        df = pd.read_csv(csv_path)
        # try to detect columns
        def pick(*names):
            for n in names:
                if n in df.columns:
                    return df[n].to_numpy(dtype=float)
            raise KeyError(f"Missing any of columns: {names}")

        # X/Y detection (case-insensitive fallback)
        columns_lower = {c.lower(): c for c in df.columns}
        def pick_ci(*names):
            for n in names:
                if n in df.columns:
                    return df[n].to_numpy(dtype=float)
                if n.lower() in columns_lower:
                    return df[columns_lower[n.lower()]].to_numpy(dtype=float)
            raise KeyError(f"Missing any of columns: {names}")

        self.x = pick_ci('x', 'X', 'map_x', 'pos_x', 'east', 'utm_x')
        self.y = pick_ci('y', 'Y', 'map_y', 'pos_y', 'north', 'utm_y')

        # s (arc length). If not provided, compute from (x,y)
        try:
            self.s = pick('s_m', 's', 'arc_length')
        except KeyError:
            dx = np.gradient(self.x)
            dy = np.gradient(self.y)
            ds = np.hypot(dx, dy)
            # trapezoidal cumulative
            self.s = np.cumsum(np.hstack([[0.0], 0.5 * (ds[1:] + ds[:-1])]))

        # heading and curvature; fallback compute if missing
        try:
            self.psi = pick('heading_rad', 'yaw_rad', 'psi_rad')
        except KeyError:
            xs = np.gradient(self.x, self.s)
            ys = np.gradient(self.y, self.s)
            self.psi = np.arctan2(ys, xs)

        try:
            self.kappa = pick('curvature_1pm', 'kappa', 'curvature')
        except KeyError:
            xs = np.gradient(self.x, self.s)
            ys = np.gradient(self.y, self.s)
            xss = np.gradient(xs, self.s)
            yss = np.gradient(ys, self.s)
            den = np.power(xs*xs + ys*ys, 1.5)
            den[den < 1e-9] = 1e-9
            self.kappa = (xs * yss - ys * xss) / den

        try:
            self.v_ref = pick('v_ref_mps', 'v_mps', 'speed_mps')
        except KeyError:
            # no speed column; set a constant placeholder (10 m/s)
            self.v_ref = np.full_like(self.s, 10.0)

        # precompute gradient dv/ds for feed-forward accel: a_x = v * dv/ds
        self.dv_ds = np.gradient(self.v_ref, self.s)

        # ensure s increasing from 0 .. s_last
        self.s_total = float(self.s[-1]) if len(self.s) > 0 else 1.0

    def _wrap_s(self, s_query: float) -> float:
        if self.s_total <= 0.0:
            return s_query
        return s_query % self.s_total

    def interp(self, s_query: float) -> Tuple[float, float, float]:
        """Return (psi_d, kappa_d, v_ref) at s_query (with wrap)."""
        sq = self._wrap_s(s_query)
        psi_d = np.interp(sq, self.s, self.psi)
        # keep psi continuous by small unwrap around local area (optional)
        kappa_d = np.interp(sq, self.s, self.kappa)
        v_d = np.interp(sq, self.s, self.v_ref)
        return psi_d, kappa_d, v_d

    def ax_ff(self, s_query: float, v: float) -> float:
        sq = self._wrap_s(s_query)
        dvds = np.interp(sq, self.s, self.dv_ds)
        return v * dvds

    def nearest_index(self, x: float, y: float, hint: int = None, window: int = 50) -> int:
        """Linear nearest search with optional window around hint index."""
        if hint is None:
            idxs = np.arange(len(self.x))
        else:
            i0 = max(0, hint - window)
            i1 = min(len(self.x), hint + window + 1)
            idxs = np.arange(i0, i1)
        dx = self.x[idxs] - x
        dy = self.y[idxs] - y
        j = int(np.argmin(dx * dx + dy * dy))
        return int(idxs[j])


class TrajFollowerNode(Node):
    def __init__(self):
        super().__init__('traj_follower')

        # Parameters
        self.declare_parameter('traj_csv', '/mnt/data/trajectory_with_speed.csv')
        self.declare_parameter('wheelbase', 1.05)
        self.declare_parameter('steer_topic', '/cmd/steer')
        self.declare_parameter('accel_topic', '/cmd/accel')
        self.declare_parameter('odom_topic', '/localization/kinematic_state')
        # control params
        self.declare_parameter('ke', 1.0)
        self.declare_parameter('kpsi', 1.5)
        self.declare_parameter('eps_v', 0.1)
        self.declare_parameter('l0', 1.0)
        self.declare_parameter('tau', 0.15)
        self.declare_parameter('Lmin', 0.8)
        self.declare_parameter('Lmax', 6.0)
        self.declare_parameter('max_steer_rate_degps', 400.0)
        self.declare_parameter('ax_kp', 1.0)
        self.declare_parameter('ax_ki', 0.2)
        self.declare_parameter('ax_kd', 0.0)
        self.declare_parameter('ax_i_min', -2.0)
        self.declare_parameter('ax_i_max', 2.0)
        self.declare_parameter('ax_min', -3.0)
        self.declare_parameter('ax_max', 2.0)
        self.declare_parameter('ctrl_hz', 50.0)

        # Load params
        traj_path = self.get_parameter('traj_csv').get_parameter_value().string_value
        L = self.get_parameter('wheelbase').get_parameter_value().double_value
        steer_topic = self.get_parameter('steer_topic').get_parameter_value().string_value
        accel_topic = self.get_parameter('accel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.L = L
        self.gains = Gains(
            ke=self.get_parameter('ke').get_parameter_value().double_value,
            kpsi=self.get_parameter('kpsi').get_parameter_value().double_value,
            eps_v=self.get_parameter('eps_v').get_parameter_value().double_value,
        )
        self.look = Lookahead(
            l0=self.get_parameter('l0').get_parameter_value().double_value,
            tau=self.get_parameter('tau').get_parameter_value().double_value,
            Lmin=self.get_parameter('Lmin').get_parameter_value().double_value,
            Lmax=self.get_parameter('Lmax').get_parameter_value().double_value,
        )
        self.max_steer_rate_degps = self.get_parameter('max_steer_rate_degps').get_parameter_value().double_value

        self.pid = PID(
            kp=self.get_parameter('ax_kp').get_parameter_value().double_value,
            ki=self.get_parameter('ax_ki').get_parameter_value().double_value,
            kd=self.get_parameter('ax_kd').get_parameter_value().double_value,
            i_min=self.get_parameter('ax_i_min').get_parameter_value().double_value,
            i_max=self.get_parameter('ax_i_max').get_parameter_value().double_value,
        )
        self.ax_min = self.get_parameter('ax_min').get_parameter_value().double_value
        self.ax_max = self.get_parameter('ax_max').get_parameter_value().double_value

        self.ctrl_dt = 1.0 / max(1e-3, self.get_parameter('ctrl_hz').get_parameter_value().double_value)

        # Load trajectory
        self.traj = Trajectory(traj_path)
        self.get_logger().info(f"Trajectory loaded: {traj_path} (N={len(self.traj.s)})")

        # Publishers / Subscribers
        self.pub_steer = self.create_publisher(Float32, steer_topic, 10)
        self.pub_accel = self.create_publisher(Float32, accel_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        # internal state
        self._last_idx = None
        self._last_delta = 0.0  # rad
        self._last_time = None
        self.pid.reset()

    # --- Helpers ---
    def _rate_limit(self, delta_cmd: float, dt: float) -> float:
        if dt <= 0.0:
            return delta_cmd
        # limit steering rate in deg/s
        ddeg = math.degrees(delta_cmd - self._last_delta) / dt
        if abs(ddeg) > self.max_steer_rate_degps:
            ddeg = math.copysign(self.max_steer_rate_degps, ddeg)
        new_delta = self._last_delta + math.radians(ddeg) * dt
        return new_delta

    def _ax_to_pedal(self, ax_cmd: float) -> float:
        """Map longitudinal accel [m/s^2] to a generic pedal [0..1].
        Replace with your calibrated map if available.
        """
        # Simple linear map as placeholder: ax_max -> 1.0, ax_min -> 0.0
        ax_cmd = max(self.ax_min, min(self.ax_max, ax_cmd))
        p = (ax_cmd - self.ax_min) / max(1e-6, (self.ax_max - self.ax_min))
        return float(p)

    # --- Core ---
    def odom_cb(self, msg: Odometry):
        # extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # speed from twist if available
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = float(math.hypot(vx, vy))

        now = msg.header.stamp
        t_now = Time.from_msg(now)
        if self._last_time is None:
            dt = self.ctrl_dt
        else:
            dt = max(1e-3, (t_now - self._last_time).nanoseconds * 1e-9)
        self._last_time = t_now

        # nearest on trajectory
        idx = self.traj.nearest_index(x, y, hint=self._last_idx)
        self._last_idx = idx
        xr = self.traj.x[idx]
        yr = self.traj.y[idx]
        sr = self.traj.s[idx]
        psi_d = self.traj.psi[idx]

        # Frenet errors
        # tangent and normal at ref
        nx = -math.sin(psi_d)
        ny =  math.cos(psi_d)
        ex = x - xr
        ey = y - yr
        e_y = ex * nx + ey * ny               # signed lateral error
        e_psi = wrap_angle(yaw - psi_d)       # yaw error

        # lookahead
        Ld = self.look.l0 + self.look.tau * v
        Ld = max(self.look.Lmin, min(self.look.Lmax, Ld))
        sp = sr + Ld

        # ref values at lookahead
        psi_p, kappa_p, v_ref = self.traj.interp(sp)

        # steering: FF + Stanley + yaw term
        delta_ff = math.atan(self.L * kappa_p)
        delta_y = math.atan(self.gains.ke * e_y / max(self.gains.eps_v, v))
        delta_psi = self.gains.kpsi * e_psi
        delta_cmd = delta_ff + delta_y + delta_psi

        # rate limit
        delta_cmd = self._rate_limit(delta_cmd, dt)

        # longitudinal: FF + PID on speed
        ax_ff = self.traj.ax_ff(sp, v)
        e_v = v_ref - v
        ax_fb = self.pid.step(e_v, dt)
        ax_cmd = ax_ff + ax_fb
        ax_cmd = max(self.ax_min, min(self.ax_max, ax_cmd))

        # publish
        m_steer = Float32(); m_steer.data = float(delta_cmd)
        m_accel = Float32(); m_accel.data = float(ax_cmd)   # or self._ax_to_pedal(ax_cmd)
        self.pub_steer.publish(m_steer)
        self.pub_accel.publish(m_accel)

        # Optional: debug in logs at low rate
        # self.get_logger().info(f"ey={e_y:.3f} epsi={e_psi:.3f} v={v:.2f}->{v_ref:.2f} ax={ax_cmd:.2f} delta={delta_cmd:.3f}")


def main():
    rclpy.init()
    node = TrajFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
