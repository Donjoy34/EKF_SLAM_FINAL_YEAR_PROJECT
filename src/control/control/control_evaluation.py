import csv
import math
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import WaypointArrayStamped, WheelSpeeds
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ControlEvaluation(Node):
	def __init__(self) -> None:
		super().__init__('control_evaluation')

		# Match key parameters used by control_node.
		self.declare_parameter('static_lookahead_idx', 6)
		self.declare_parameter('wheel_radius_m', 0.2525)
		self.declare_parameter('steer_limit_deg', 60.0)
		self.declare_parameter('eval_window_sec', 20.0)
		self.declare_parameter('plot_period_sec', 0.2)
		self.declare_parameter('csv_log_enable', True)
		self.declare_parameter('csv_log_path', 'control_eval.csv')

		self.static_lookahead_idx = int(self.get_parameter('static_lookahead_idx').value)
		self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
		self.steer_limit_deg = float(self.get_parameter('steer_limit_deg').value)
		self.eval_window_sec = float(self.get_parameter('eval_window_sec').value)
		self.plot_period_sec = float(self.get_parameter('plot_period_sec').value)
		self.csv_log_enable = bool(self.get_parameter('csv_log_enable').value)
		self.csv_log_path = str(self.get_parameter('csv_log_path').value)

		self.speed_mps = 0.0
		self.last_cmd_steer_rad: Optional[float] = None
		self.last_cmd_accel: Optional[float] = None
		self.t0: Optional[float] = None
		self.csv_file = None
		self.csv_writer = None
		self.last_metrics = {
			'steer_peak': 0.0,
			'steer_mean': 0.0,
			'steer_settle': 0.0,
			'steer_iae': 0.0,
			'steer_ise': 0.0,
			'steer_itae': 0.0,
			'steer_cmd_max': 0.0,
			'steer_sat_frac': 0.0,
		}

		# Time series: (t, steering_error_deg, steer_cmd_deg, accel_cmd)
		self.samples: List[Tuple[float, float, float, float]] = []

		self.create_subscription(WaypointArrayStamped, '/trajectory', self.on_trajectory, 1)
		self.create_subscription(AckermannDriveStamped, '/cmd', self.on_cmd, 1)
		self.create_subscription(WheelSpeeds, '/ros_can/wheel_speeds', self.on_wheel_speeds, qos_profile_sensor_data)

		if self.csv_log_enable:
			self.csv_file = open(self.csv_log_path, 'w', newline='')
			self.csv_writer = csv.writer(self.csv_file)
			self.csv_writer.writerow([
				't',
				't_rel',
				'steer_error_deg',
				'raw_deg',
				'steer_cmd_deg',
				'accel_cmd',
				'steer_peak_deg',
				'steer_mean_abs_deg',
				'steer_settle_s',
				'steer_iae',
				'steer_ise',
				'steer_itae',
				'steer_cmd_max_deg',
				'steer_sat_frac',
			])

		self.fig, (self.ax_err, self.ax_cmd) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
		self.fig.suptitle('Control Evaluation')
		self.ax_err.set_ylabel('Steer Error [deg]')
		self.ax_cmd.set_ylabel('Cmd (deg, m/s^2)')
		self.ax_cmd.set_xlabel('Time [s]')

		self.err_line, = self.ax_err.plot([], [], 'b-', label='Steer Error')
		self.steer_cmd_line, = self.ax_cmd.plot([], [], 'r-', label='Steer Cmd [deg]')
		self.accel_cmd_line, = self.ax_cmd.plot([], [], 'm-', label='Accel Cmd')

		self.ax_err.grid(True, alpha=0.3)
		self.ax_cmd.grid(True, alpha=0.3)
		self.ax_err.legend(loc='best')
		self.ax_cmd.legend(loc='best')

		self.metrics_text = self.fig.text(0.02, 0.01, '', fontsize=9, family='monospace')

		plt.ion()
		plt.show(block=False)

		self.create_timer(max(0.05, self.plot_period_sec), self.update_plot)

	def on_cmd(self, msg: AckermannDriveStamped) -> None:
		self.last_cmd_steer_rad = float(msg.drive.steering_angle)
		self.last_cmd_accel = float(msg.drive.acceleration)

	def on_wheel_speeds(self, msg: WheelSpeeds) -> None:
		rear_rpm = 0.5 * (abs(msg.lb_speed) + abs(msg.rb_speed))
		wheel_circ = 2.0 * math.pi * self.wheel_radius_m
		self.speed_mps = (rear_rpm / 60.0) * wheel_circ

	def on_trajectory(self, msg: WaypointArrayStamped) -> None:
		if not msg.waypoints:
			return

		now = self.get_clock().now().nanoseconds * 1e-9
		if self.t0 is None:
			self.t0 = now
		idx = self.static_lookahead_idx
		look_wp = msg.waypoints[idx] if idx < len(msg.waypoints) else msg.waypoints[-1]

		raw_deg = math.degrees(math.atan2(look_wp.position.y, look_wp.position.x))
		steer_error = abs(raw_deg)

		steer_cmd_deg = math.degrees(self.last_cmd_steer_rad) if self.last_cmd_steer_rad is not None else 0.0
		accel_cmd = self.last_cmd_accel if self.last_cmd_accel is not None else 0.0

		self.samples.append((now, steer_error, steer_cmd_deg, accel_cmd))
		self.trim_samples(now)
		self.write_csv_sample(
			now,
			raw_deg,
			steer_error,
			steer_cmd_deg,
			accel_cmd,
		)

	def write_csv_sample(
		self,
		now: float,
		raw_deg: float,
		steer_error: float,
		steer_cmd_deg: float,
		accel_cmd: float,
	) -> None:
		if self.csv_writer is None or self.t0 is None:
			return
		t_rel = now - self.t0
		self.csv_writer.writerow([
			f'{now:.6f}',
			f'{t_rel:.6f}',
			f'{steer_error:.6f}',
			f'{raw_deg:.6f}',
			f'{steer_cmd_deg:.6f}',
			f'{accel_cmd:.6f}',
			f"{self.last_metrics['steer_peak']:.6f}",
			f"{self.last_metrics['steer_mean']:.6f}",
			f"{self.last_metrics['steer_settle']:.6f}",
			f"{self.last_metrics['steer_iae']:.6f}",
			f"{self.last_metrics['steer_ise']:.6f}",
			f"{self.last_metrics['steer_itae']:.6f}",
			f"{self.last_metrics['steer_cmd_max']:.6f}",
			f"{self.last_metrics['steer_sat_frac']:.6f}",
		])
		self.csv_file.flush()

	def trim_samples(self, now: float) -> None:
		return

	def compute_integral_metrics(self, t: np.ndarray, e: np.ndarray) -> Tuple[float, float, float]:
		if t.size < 2:
			return 0.0, 0.0, 0.0
		dt = np.diff(t)
		e_mid = 0.5 * (e[:-1] + e[1:])
		iae = float(np.sum(np.abs(e_mid) * dt))
		ise = float(np.sum((e_mid ** 2) * dt))
		itae = float(np.sum(np.abs(e_mid) * (t[1:] - t[0]) * dt))
		return iae, ise, itae

	def compute_time_metrics(self, t: np.ndarray, e: np.ndarray, eps: float) -> Tuple[float, float, float]:
		if t.size == 0:
			return 0.0, 0.0, 0.0
		peak = float(np.max(np.abs(e)))
		mean_abs = float(np.mean(np.abs(e)))

		settling_time = 0.0
		if t.size > 5:
			within = np.abs(e) <= eps
			# Find the earliest time after which error stays within epsilon.
			for i in range(len(within)):
				if np.all(within[i:]):
					settling_time = float(t[i] - t[0])
					break
		return peak, mean_abs, settling_time

	def update_plot(self) -> None:
		if not self.samples:
			return

		data = np.array(self.samples, dtype=float)
		t = data[:, 0] - (self.t0 if self.t0 is not None else data[0, 0])
		steer_err = data[:, 1]
		steer_cmd = data[:, 2]
		accel_cmd = data[:, 3]

		self.err_line.set_data(t, steer_err)
		self.steer_cmd_line.set_data(t, steer_cmd)
		self.accel_cmd_line.set_data(t, accel_cmd)

		for ax in (self.ax_err, self.ax_cmd):
			ax.relim()
			ax.autoscale_view()

		iae_s, ise_s, itae_s = self.compute_integral_metrics(t, steer_err)

		peak_s, mean_s, settle_s = self.compute_time_metrics(t, steer_err, eps=2.0)

		steer_limit = self.steer_limit_deg
		sat_frac = 0.0
		if t.size > 0:
			sat_frac = float(np.mean(np.abs(steer_cmd) >= steer_limit))

		self.last_metrics.update({
			'steer_peak': peak_s,
			'steer_mean': mean_s,
			'steer_settle': settle_s,
			'steer_iae': iae_s,
			'steer_ise': ise_s,
			'steer_itae': itae_s,
			'steer_cmd_max': float(np.max(np.abs(steer_cmd))) if steer_cmd.size else 0.0,
			'steer_sat_frac': sat_frac,
		})

		metrics = (
			f'STEER: peak={peak_s:.2f} deg, mean|e|={mean_s:.2f} deg, settle={settle_s:.2f} s, '
			f'IAE={iae_s:.2f}, ISE={ise_s:.2f}, ITAE={itae_s:.2f}\n'
			f'CTRL: steer|max|={np.max(np.abs(steer_cmd)):.2f} deg, sat%={100.0 * sat_frac:.1f}%'
		)
		self.metrics_text.set_text(metrics)

		self.fig.canvas.draw_idle()
		self.fig.canvas.flush_events()
		plt.pause(0.001)


def main() -> None:
	rclpy.init()
	node = ControlEvaluation()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		if node.csv_file is not None:
			node.csv_file.close()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
