import csv
import math
import os
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import WaypointArrayStamped, WheelSpeeds, CarState
from rclpy.node import Node


class PidEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("pid_evaluator")

        self.declare_parameter("trajectory_topic", "/trajectory")
        self.declare_parameter("cmd_topic", "/cmd")
        self.declare_parameter("car_state_topic", "/ground_truth/state")
        self.declare_parameter("wheel_topic", "/ros_can/wheel_speeds")
        self.declare_parameter("static_lookahead_idx", 6)
        self.declare_parameter("wheel_radius_m", 0.2525)
        self.declare_parameter("speed_timeout_sec", 0.5)

        self.declare_parameter("enable_pid_eval_csv", True)
        self.declare_parameter("pid_eval_csv_path", "~/uh-fs-ai/slam_eval/pid_eval.csv")
        self.declare_parameter("pid_eval_csv_append", False)
        self.declare_parameter("pid_eval_csv_flush_period_sec", 1.0)

        self.declare_parameter("enable_pid_eval_plot", False)
        self.declare_parameter("pid_eval_plot_period_sec", 0.25)
        self.declare_parameter("pid_eval_plot_max_points", 800)
        self.declare_parameter("pid_eval_plot_backend", "")

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.car_state_topic = str(self.get_parameter("car_state_topic").value)
        self.wheel_topic = str(self.get_parameter("wheel_topic").value)
        self.static_lookahead_idx = int(self.get_parameter("static_lookahead_idx").value)
        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.speed_timeout_sec = float(self.get_parameter("speed_timeout_sec").value)

        self.enable_pid_eval_csv = bool(self.get_parameter("enable_pid_eval_csv").value)
        self.pid_eval_csv_path = os.path.expanduser(str(self.get_parameter("pid_eval_csv_path").value))
        self.pid_eval_csv_append = bool(self.get_parameter("pid_eval_csv_append").value)
        self.pid_eval_csv_flush_period_sec = float(self.get_parameter("pid_eval_csv_flush_period_sec").value)

        self.enable_pid_eval_plot = bool(self.get_parameter("enable_pid_eval_plot").value)
        self.pid_eval_plot_period_sec = float(self.get_parameter("pid_eval_plot_period_sec").value)
        self.pid_eval_plot_max_points = int(self.get_parameter("pid_eval_plot_max_points").value)
        self.pid_eval_plot_backend = str(self.get_parameter("pid_eval_plot_backend").value)

        self.last_raw_deg: Optional[float] = None
        self.last_error_deg: Optional[float] = None
        self.last_trajectory_time: Optional[float] = None

        self.speed_mps = 0.0
        self.last_speed_time: Optional[float] = None
        self.speed_source = "NONE"
        self.last_speed_for_accel: Optional[float] = None
        self.last_accel_time: Optional[float] = None

        self.csv_file = None
        self.csv_writer = None
        self.last_flush_time = -1e9

        self.time_hist = []
        self.err_hist = []
        self.steer_hist = []
        self.accel_cmd_hist = []
        self.accel_meas_hist = []
        self.plot_ready = False
        self.plot_fig = None
        self.plot_axes = None
        self.plot_lines = None
        self.plt = None

        self.create_subscription(WaypointArrayStamped, self.trajectory_topic, self.on_trajectory, 10)
        self.create_subscription(AckermannDriveStamped, self.cmd_topic, self.on_cmd, 20)
        self.create_subscription(CarState, self.car_state_topic, self.on_car_state, 20)
        self.create_subscription(WheelSpeeds, self.wheel_topic, self.on_wheel_speeds, 20)

        if self.enable_pid_eval_csv:
            self.init_csv()

        if self.enable_pid_eval_plot:
            self.setup_plot()
            if self.plot_ready:
                self.create_timer(max(0.1, self.pid_eval_plot_period_sec), self.update_plot)

        self.get_logger().info("pid_evaluator active")

    def on_trajectory(self, msg: WaypointArrayStamped) -> None:
        wps = [[wp.position.x, wp.position.y] for wp in msg.waypoints]
        if not wps:
            return
        idx = self.static_lookahead_idx
        look_wp = wps[idx] if idx < len(wps) else wps[-1]
        raw_deg = math.degrees(math.atan2(look_wp[1], look_wp[0]))
        self.last_raw_deg = raw_deg
        self.last_error_deg = abs(raw_deg)
        self.last_trajectory_time = self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg: AckermannDriveStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        steer_rad = float(msg.drive.steering_angle)
        accel_cmd = float(msg.drive.acceleration)
        steer_deg = math.degrees(steer_rad)

        speed_fresh = self.last_speed_time is not None and (
            (now - self.last_speed_time) <= self.speed_timeout_sec
        )
        accel_meas = self.compute_accel(now)

        err_deg = self.last_error_deg if self.last_error_deg is not None else float("nan")
        raw_deg = self.last_raw_deg if self.last_raw_deg is not None else float("nan")

        self.log_csv(now, steer_deg, accel_cmd, raw_deg, err_deg, speed_fresh, accel_meas)
        self.update_history(now, err_deg, steer_deg, accel_cmd, accel_meas)

    def on_car_state(self, msg: CarState) -> None:
        self.speed_mps = abs(float(msg.twist.twist.linear.x))
        self.last_speed_time = self.get_clock().now().nanoseconds * 1e-9
        self.speed_source = "GROUND_TRUTH"

    def on_wheel_speeds(self, msg: WheelSpeeds) -> None:
        rear_rpm = 0.5 * (abs(msg.lb_speed) + abs(msg.rb_speed))
        wheel_circumference = 2.0 * math.pi * self.wheel_radius_m
        self.speed_mps = (rear_rpm / 60.0) * wheel_circumference
        self.last_speed_time = self.get_clock().now().nanoseconds * 1e-9
        self.speed_source = "WHEEL_RPM"

    def compute_accel(self, now: float) -> float:
        if self.last_accel_time is None or self.last_speed_for_accel is None:
            self.last_accel_time = now
            self.last_speed_for_accel = self.speed_mps
            return float("nan")
        dt = now - self.last_accel_time
        if dt <= 1e-6:
            return float("nan")
        accel = (self.speed_mps - self.last_speed_for_accel) / dt
        self.last_accel_time = now
        self.last_speed_for_accel = self.speed_mps
        return accel

    def init_csv(self) -> None:
        os.makedirs(os.path.dirname(self.pid_eval_csv_path), exist_ok=True)
        mode = "a" if self.pid_eval_csv_append else "w"
        self.csv_file = open(self.pid_eval_csv_path, mode, newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        if not self.pid_eval_csv_append:
            self.csv_writer.writerow([
                "t",
                "cmd_steer_deg",
                "cmd_accel",
                "raw_steer_deg",
                "pid_error_deg",
                "speed_mps",
                "speed_source",
                "speed_fresh",
                "accel_meas",
                "accel_error",
            ])

    def log_csv(
        self,
        now: float,
        steer_deg: float,
        accel_cmd: float,
        raw_deg: float,
        err_deg: float,
        speed_fresh: bool,
        accel_meas: float,
    ) -> None:
        if not self.enable_pid_eval_csv or self.csv_writer is None:
            return
        accel_error = float("nan")
        if math.isfinite(accel_meas):
            accel_error = accel_cmd - accel_meas
        self.csv_writer.writerow([
            float(now),
            float(steer_deg),
            float(accel_cmd),
            float(raw_deg),
            float(err_deg),
            float(self.speed_mps),
            str(self.speed_source),
            int(speed_fresh),
            float(accel_meas),
            float(accel_error),
        ])
        if now - self.last_flush_time >= self.pid_eval_csv_flush_period_sec:
            self.last_flush_time = now
            try:
                self.csv_file.flush()
            except Exception:
                pass

    def update_history(
        self,
        now: float,
        err_deg: float,
        steer_deg: float,
        accel_cmd: float,
        accel_meas: float,
    ) -> None:
        if not self.enable_pid_eval_plot:
            return
        self.time_hist.append(now)
        self.err_hist.append(err_deg)
        self.steer_hist.append(steer_deg)
        self.accel_cmd_hist.append(accel_cmd)
        self.accel_meas_hist.append(accel_meas)
        if len(self.time_hist) > self.pid_eval_plot_max_points:
            start = len(self.time_hist) - self.pid_eval_plot_max_points
            self.time_hist = self.time_hist[start:]
            self.err_hist = self.err_hist[start:]
            self.steer_hist = self.steer_hist[start:]
            self.accel_cmd_hist = self.accel_cmd_hist[start:]
            self.accel_meas_hist = self.accel_meas_hist[start:]

    def setup_plot(self) -> None:
        try:
            import matplotlib
            if self.pid_eval_plot_backend:
                matplotlib.use(self.pid_eval_plot_backend, force=True)
            import matplotlib.pyplot as plt

            plt.ion()
            self.plt = plt
            self.plot_fig, self.plot_axes = plt.subplots(2, 2, figsize=(11, 7.5))
            (ax_err, ax_steer), (ax_accel, ax_accel_meas) = self.plot_axes

            err_line, = ax_err.plot([], [], color="crimson", linewidth=1.6, label="PID error [deg]")
            steer_line, = ax_steer.plot([], [], color="slateblue", linewidth=1.5, label="Cmd steer [deg]")
            accel_line, = ax_accel.plot([], [], color="darkgreen", linewidth=1.5, label="Cmd accel")
            meas_line, = ax_accel_meas.plot([], [], color="darkorange", linewidth=1.5, label="Meas accel")

            for ax, title in (
                (ax_err, "Steering Error"),
                (ax_steer, "Steering Command"),
                (ax_accel, "Acceleration Command"),
                (ax_accel_meas, "Measured Acceleration"),
            ):
                ax.set_title(title)
                ax.set_xlabel("Time [s]")
                ax.grid(True, alpha=0.3)
                ax.legend(loc="best")

            self.plot_lines = (err_line, steer_line, accel_line, meas_line)
            self.plot_fig.tight_layout()
            self.plot_ready = True
            self.plt.show(block=False)
        except Exception as exc:
            self.plot_ready = False
            self.get_logger().warn(f"pid eval plot disabled: {exc}")

    def update_plot(self) -> None:
        if not self.plot_ready or self.plot_fig is None or self.plot_axes is None or self.plt is None:
            return
        if not self.time_hist:
            return
        try:
            t0 = self.time_hist[0]
            t = [ts - t0 for ts in self.time_hist]
            err_line, steer_line, accel_line, meas_line = self.plot_lines

            err_line.set_data(t, self.err_hist)
            steer_line.set_data(t, self.steer_hist)
            accel_line.set_data(t, self.accel_cmd_hist)
            meas_line.set_data(t, self.accel_meas_hist)

            for ax in self.plot_axes.flatten():
                ax.relim()
                ax.autoscale_view()

            self.plot_fig.canvas.draw_idle()
            self.plot_fig.canvas.flush_events()
            self.plt.pause(0.001)
        except Exception as exc:
            self.plot_ready = False
            self.get_logger().warn(f"pid eval plot disabled after error: {exc}")

    def close(self) -> None:
        if self.csv_file is not None:
            try:
                self.csv_file.flush()
            except Exception:
                pass
            try:
                self.csv_file.close()
            except Exception:
                pass
        self.csv_file = None
        self.csv_writer = None

        if self.plot_ready and self.plt is not None:
            try:
                self.plt.close("all")
            except Exception:
                pass


def main() -> None:
    rclpy.init()
    node = PidEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
