import csv
import json
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray


@dataclass
class Pose2DStamped:
    t: float
    x: float
    y: float
    yaw: float


class SlamEvaluator(Node):
    def __init__(self) -> None:
        super().__init__('slam_evaluator')

        self.declare_parameter('slam_odom_topic', '/slam/odom')
        self.declare_parameter('gt_odom_topic', '/ground_truth/odom')
        self.declare_parameter('landmark_topic', '/slam/landmarks')
        self.declare_parameter('output_dir', '~/uh-fs-ai/slam_eval')
        self.declare_parameter('run_name', 'ekf_slam_run')
        self.declare_parameter('rpe_delta_samples', 20)
        self.declare_parameter('auto_export_on_shutdown', True)
        self.declare_parameter('enable_realtime_csv', True)
        self.declare_parameter('realtime_csv_name', 'realtime_metrics.csv')
        self.declare_parameter('realtime_time_sync_sec', 0.2)
        self.declare_parameter('realtime_allow_unsynced', True)
        self.declare_parameter('realtime_align_gt', True)
        self.declare_parameter('cov_eig_floor', 1e-6)
        self.declare_parameter('csv_append', False)
        self.declare_parameter('csv_flush_period_sec', 1.0)
        self.declare_parameter('enable_realtime_plot', True)
        self.declare_parameter('realtime_plot_period_sec', 0.25)
        self.declare_parameter('realtime_plot_max_points', 800)
        self.declare_parameter('realtime_plot_backend', '')
        self.declare_parameter('gt_pose_cov_x', 0.02)
        self.declare_parameter('gt_pose_cov_y', 0.02)
        self.declare_parameter('gt_pose_cov_yaw', 0.01)
        self.declare_parameter('innovation_topic', '/ekf_slam/innovations')

        self.slam_odom_topic = str(self.get_parameter('slam_odom_topic').value)
        self.gt_odom_topic = str(self.get_parameter('gt_odom_topic').value)
        self.landmark_topic = str(self.get_parameter('landmark_topic').value)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.run_name = str(self.get_parameter('run_name').value)
        self.rpe_delta_samples = int(self.get_parameter('rpe_delta_samples').value)
        self.auto_export_on_shutdown = bool(self.get_parameter('auto_export_on_shutdown').value)
        self.enable_realtime_csv = bool(self.get_parameter('enable_realtime_csv').value)
        self.realtime_csv_name = str(self.get_parameter('realtime_csv_name').value)
        self.realtime_time_sync_sec = float(self.get_parameter('realtime_time_sync_sec').value)
        self.realtime_allow_unsynced = bool(self.get_parameter('realtime_allow_unsynced').value)
        self.realtime_align_gt = bool(self.get_parameter('realtime_align_gt').value)
        self.cov_eig_floor = float(self.get_parameter('cov_eig_floor').value)
        self.csv_append = bool(self.get_parameter('csv_append').value)
        self.csv_flush_period_sec = float(self.get_parameter('csv_flush_period_sec').value)
        self.enable_realtime_plot = bool(self.get_parameter('enable_realtime_plot').value)
        self.realtime_plot_period_sec = float(self.get_parameter('realtime_plot_period_sec').value)
        self.realtime_plot_max_points = int(self.get_parameter('realtime_plot_max_points').value)
        self.realtime_plot_backend = str(self.get_parameter('realtime_plot_backend').value)
        self.gt_pose_cov_x = float(self.get_parameter('gt_pose_cov_x').value)
        self.gt_pose_cov_y = float(self.get_parameter('gt_pose_cov_y').value)
        self.gt_pose_cov_yaw = float(self.get_parameter('gt_pose_cov_yaw').value)
        self.innovation_topic = str(self.get_parameter('innovation_topic').value)

        self.slam_poses: List[Pose2DStamped] = []
        self.gt_poses: List[Pose2DStamped] = []
        self.landmarks_xy: List[Tuple[float, float]] = []
        self.last_slam_pose: Optional[Pose2DStamped] = None
        self.last_gt_pose: Optional[Pose2DStamped] = None
        self.last_slam_cov: Optional[np.ndarray] = None
        self.rt_align_ready = False
        self.rt_align_rot = np.eye(2, dtype=float)
        self.rt_align_trans = np.zeros((2,), dtype=float)
        self.rt_csv_file: Optional[object] = None
        self.rt_csv_writer: Optional[csv.writer] = None
        self.rt_last_flush_time = -1e9
        self.gt_pose_cov = np.diag([self.gt_pose_cov_x, self.gt_pose_cov_y, self.gt_pose_cov_yaw]).astype(float)
        self.last_innov_time: Optional[float] = None
        self.last_innov_nis: Optional[float] = None
        self.last_innov_type: Optional[int] = None
        self.rt_time_history: List[float] = []
        self.rt_pos_err_history: List[float] = []
        self.rt_yaw_err_deg_history: List[float] = []
        self.rt_nees_history: List[float] = []
        self.rt_nis_history: List[float] = []
        self.rt_plot_ready = False
        self.rt_fig = None
        self.rt_axes = None
        self.rt_lines = None
        self.rt_pl = None

        self.create_subscription(Odometry, self.slam_odom_topic, self.on_slam_odom, 50)
        self.create_subscription(Odometry, self.gt_odom_topic, self.on_gt_odom, 50)
        self.create_subscription(MarkerArray, self.landmark_topic, self.on_landmarks, 10)
        self.create_subscription(Float64MultiArray, self.innovation_topic, self.on_innovation, 50)
        self.export_srv = self.create_service(Trigger, '/slam_eval/export', self.on_export)

        if self.enable_realtime_csv:
            self.init_realtime_csv()

        if self.enable_realtime_plot:
            self.setup_realtime_plot()
            if self.rt_plot_ready:
                self.create_timer(max(0.1, self.realtime_plot_period_sec), self.update_realtime_plot)

        self.get_logger().info('slam_evaluator active: collecting slam/gt trajectories and landmarks')

    def on_slam_odom(self, msg: Odometry) -> None:
        t = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        pose = Pose2DStamped(
            t=t,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        )
        self.slam_poses.append(pose)
        self.last_slam_pose = pose
        self.last_slam_cov = self.extract_pose_covariance(msg)
        self.maybe_log_realtime()

    def on_gt_odom(self, msg: Odometry) -> None:
        t = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        pose = Pose2DStamped(
            t=t,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        )
        self.gt_poses.append(pose)
        self.last_gt_pose = pose
        self.maybe_log_realtime()

    def on_landmarks(self, msg: MarkerArray) -> None:
        points = []
        for marker in msg.markers:
            if marker.action != marker.DELETE:
                points.append((float(marker.pose.position.x), float(marker.pose.position.y)))
        if points:
            self.landmarks_xy = points

    def on_export(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, out_msg = self.export_all()
        response.success = ok
        response.message = out_msg
        return response

    def init_realtime_csv(self) -> None:
        os.makedirs(self.output_dir, exist_ok=True)
        run_dir = os.path.join(self.output_dir, self.run_name)
        os.makedirs(run_dir, exist_ok=True)
        path = os.path.join(run_dir, self.realtime_csv_name)
        mode = 'a' if self.csv_append else 'w'
        self.rt_csv_file = open(path, mode, newline='', encoding='utf-8')
        self.rt_csv_writer = csv.writer(self.rt_csv_file)
        if not self.csv_append:
            self.rt_csv_writer.writerow([
                't_slam', 't_gt', 'dt',
                'slam_x', 'slam_y', 'slam_yaw_rad',
                'gt_x', 'gt_y', 'gt_yaw_rad',
                'err_x', 'err_y', 'err_pos', 'err_yaw_rad',
                'nees_pose', 'nis_pose', 'nis_source', 'innov_type',
                'cov_xx', 'cov_xy', 'cov_xyaw',
                'cov_yy', 'cov_yyaw', 'cov_yawyaw',
            ])

    def close_realtime_csv(self) -> None:
        if self.rt_csv_file is not None:
            try:
                self.rt_csv_file.flush()
            except Exception:
                pass
            try:
                self.rt_csv_file.close()
            except Exception:
                pass
        self.rt_csv_file = None
        self.rt_csv_writer = None

    def export_all(self) -> Tuple[bool, str]:
        if len(self.slam_poses) < 10 or len(self.gt_poses) < 10:
            return False, 'not enough samples: need both slam and ground-truth odometry streams'

        try:
            os.makedirs(self.output_dir, exist_ok=True)
            run_dir = os.path.join(self.output_dir, self.run_name)
            os.makedirs(run_dir, exist_ok=True)

            aligned = self.align_ground_truth_to_slam(self.slam_poses, self.gt_poses)
            if aligned is None:
                return False, 'failed to align gt to slam timestamps'

            slam_arr, gt_arr = aligned
            metrics = self.compute_metrics(slam_arr, gt_arr)

            self.save_time_series_csv(run_dir, slam_arr, gt_arr)
            self.save_metrics_json(run_dir, metrics)
            self.generate_plots(run_dir, slam_arr, gt_arr, metrics, self.landmarks_xy)

            return True, f'exported evaluation to {run_dir}'
        except Exception as exc:
            self.get_logger().error(f'export failed: {exc}')
            return False, f'export failed: {exc}'

    def align_ground_truth_to_slam(
        self,
        slam_poses: List[Pose2DStamped],
        gt_poses: List[Pose2DStamped],
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        slam_t = np.array([p.t for p in slam_poses], dtype=float)
        slam_x = np.array([p.x for p in slam_poses], dtype=float)
        slam_y = np.array([p.y for p in slam_poses], dtype=float)
        slam_yaw = np.unwrap(np.array([p.yaw for p in slam_poses], dtype=float))

        gt_t = np.array([p.t for p in gt_poses], dtype=float)
        gt_x = np.array([p.x for p in gt_poses], dtype=float)
        gt_y = np.array([p.y for p in gt_poses], dtype=float)
        gt_yaw = np.unwrap(np.array([p.yaw for p in gt_poses], dtype=float))

        t_min = max(float(np.min(slam_t)), float(np.min(gt_t)))
        t_max = min(float(np.max(slam_t)), float(np.max(gt_t)))
        mask = (slam_t >= t_min) & (slam_t <= t_max)
        if np.count_nonzero(mask) < 10:
            return None

        slam_t = slam_t[mask]
        slam_x = slam_x[mask]
        slam_y = slam_y[mask]
        slam_yaw = slam_yaw[mask]

        gt_x_i = np.interp(slam_t, gt_t, gt_x)
        gt_y_i = np.interp(slam_t, gt_t, gt_y)
        gt_yaw_i = np.interp(slam_t, gt_t, gt_yaw)

        slam_arr = np.column_stack((slam_t, slam_x, slam_y, slam_yaw))
        gt_arr = np.column_stack((slam_t, gt_x_i, gt_y_i, gt_yaw_i))
        return slam_arr, gt_arr

    def maybe_log_realtime(self) -> None:
        if not self.enable_realtime_csv:
            return
        if self.rt_csv_writer is None:
            return
        if self.last_slam_pose is None or self.last_gt_pose is None:
            return

        dt = abs(self.last_slam_pose.t - self.last_gt_pose.t)
        if dt > self.realtime_time_sync_sec and not self.realtime_allow_unsynced:
            return

        gt_pose = self.last_gt_pose
        if self.realtime_align_gt:
            if not self.rt_align_ready:
                self.init_realtime_alignment(self.last_slam_pose, self.last_gt_pose)
            if self.rt_align_ready:
                gt_pose = self.align_gt_pose(self.last_gt_pose)

        err_x = float(self.last_slam_pose.x - gt_pose.x)
        err_y = float(self.last_slam_pose.y - gt_pose.y)
        err_pos = float(math.hypot(err_x, err_y))
        err_yaw = float(self.normalize_angle(self.last_slam_pose.yaw - gt_pose.yaw))

        nees_pose = float('nan')
        nis_pose = float('nan')
        nis_source = 0
        innov_type = -1
        cov_vals = [float('nan')] * 6
        if self.last_slam_cov is not None:
            cov_vals = [
                float(self.last_slam_cov[0, 0]),
                float(self.last_slam_cov[0, 1]),
                float(self.last_slam_cov[0, 2]),
                float(self.last_slam_cov[1, 1]),
                float(self.last_slam_cov[1, 2]),
                float(self.last_slam_cov[2, 2]),
            ]
            err_vec = np.array([[err_x], [err_y], [err_yaw]], dtype=float)
            inv_cov = self.safe_inv_covariance(self.last_slam_cov)
            if inv_cov is not None:
                nees_pose = float(err_vec.T @ inv_cov @ err_vec)
            s_cov = self.last_slam_cov + self.gt_pose_cov
            inv_s_cov = self.safe_inv_covariance(s_cov)
            if inv_s_cov is not None:
                nis_pose = float(err_vec.T @ inv_s_cov @ err_vec)

        if self.last_innov_time is not None and self.last_innov_nis is not None:
            if abs(self.last_slam_pose.t - self.last_innov_time) <= self.realtime_time_sync_sec:
                nis_pose = float(self.last_innov_nis)
                nis_source = 1
                innov_type = int(self.last_innov_type) if self.last_innov_type is not None else -1

        self.rt_csv_writer.writerow([
            float(self.last_slam_pose.t),
            float(self.last_gt_pose.t),
            float(dt),
            float(self.last_slam_pose.x), float(self.last_slam_pose.y), float(self.last_slam_pose.yaw),
            float(gt_pose.x), float(gt_pose.y), float(gt_pose.yaw),
            err_x, err_y, err_pos, err_yaw,
            nees_pose, nis_pose, nis_source, innov_type,
            cov_vals[0], cov_vals[1], cov_vals[2], cov_vals[3], cov_vals[4], cov_vals[5],
        ])

        self.rt_time_history.append(float(self.last_slam_pose.t))
        self.rt_pos_err_history.append(err_pos)
        self.rt_yaw_err_deg_history.append(math.degrees(err_yaw))
        self.rt_nees_history.append(nees_pose)
        self.rt_nis_history.append(nis_pose)
        if len(self.rt_time_history) > self.realtime_plot_max_points:
            start = len(self.rt_time_history) - self.realtime_plot_max_points
            self.rt_time_history = self.rt_time_history[start:]
            self.rt_pos_err_history = self.rt_pos_err_history[start:]
            self.rt_yaw_err_deg_history = self.rt_yaw_err_deg_history[start:]
            self.rt_nees_history = self.rt_nees_history[start:]
            self.rt_nis_history = self.rt_nis_history[start:]

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.rt_last_flush_time >= self.csv_flush_period_sec:
            self.rt_last_flush_time = now_sec
            try:
                self.rt_csv_file.flush()
            except Exception:
                pass

    def setup_realtime_plot(self) -> None:
        try:
            import matplotlib
            if self.realtime_plot_backend:
                matplotlib.use(self.realtime_plot_backend, force=True)
            import matplotlib.pyplot as plt

            plt.ion()
            self.get_logger().info(f'realtime plot backend: {matplotlib.get_backend()}')
            self.rt_pl = plt
            self.rt_fig, self.rt_axes = plt.subplots(2, 2, figsize=(11, 7.5))
            (ax_pos, ax_yaw), (ax_nees, ax_nis) = self.rt_axes

            pos_line, = ax_pos.plot([], [], color='crimson', linewidth=1.6, label='Pos Error [m]')
            yaw_line, = ax_yaw.plot([], [], color='darkorange', linewidth=1.4, label='Yaw Error [deg]')
            nees_line, = ax_nees.plot([], [], color='teal', linewidth=1.4, label='NEES (pose)')
            nis_line, = ax_nis.plot([], [], color='slateblue', linewidth=1.4, label='NIS (pose)')

            for ax, title in (
                (ax_pos, 'Position Error'),
                (ax_yaw, 'Yaw Error'),
                (ax_nees, 'NEES'),
                (ax_nis, 'NIS'),
            ):
                ax.set_title(title)
                ax.set_xlabel('Time [s]')
                ax.grid(True, alpha=0.3)
                ax.legend(loc='best')

            self.rt_lines = (pos_line, yaw_line, nees_line, nis_line)
            self.rt_fig.tight_layout()
            self.rt_plot_ready = True
            self.rt_pl.show(block=False)
            self.rt_fig.canvas.draw_idle()
            self.rt_fig.canvas.flush_events()
            self.rt_pl.pause(0.001)
            self.get_logger().info('realtime plot window created')
        except Exception as exc:
            self.rt_plot_ready = False
            self.get_logger().warn(f'realtime plot disabled: {exc}')

    def update_realtime_plot(self) -> None:
        if not self.rt_plot_ready or self.rt_fig is None or self.rt_axes is None or self.rt_pl is None:
            return
        if not self.rt_time_history:
            try:
                self.rt_fig.canvas.draw_idle()
                self.rt_fig.canvas.flush_events()
                self.rt_pl.pause(0.001)
            except Exception as exc:
                self.rt_plot_ready = False
                self.get_logger().warn(f'realtime plot disabled after error: {exc}')
            return

        try:
            t0 = self.rt_time_history[0]
            t = np.array(self.rt_time_history, dtype=float) - t0
            pos = np.array(self.rt_pos_err_history, dtype=float)
            yaw = np.array(self.rt_yaw_err_deg_history, dtype=float)
            nees = np.array(self.rt_nees_history, dtype=float)
            nis = np.array(self.rt_nis_history, dtype=float)

            pos_line, yaw_line, nees_line, nis_line = self.rt_lines
            pos_line.set_data(t, pos)
            yaw_line.set_data(t, yaw)
            nees_line.set_data(t, nees)
            nis_line.set_data(t, nis)

            for ax in self.rt_axes.flatten():
                ax.relim()
                ax.autoscale_view()

            self.rt_fig.canvas.draw_idle()
            self.rt_fig.canvas.flush_events()
            self.rt_pl.pause(0.001)
        except Exception as exc:
            self.rt_plot_ready = False
            self.get_logger().warn(f'realtime plot disabled after error: {exc}')

    def init_realtime_alignment(self, slam_pose: Pose2DStamped, gt_pose: Pose2DStamped) -> None:
        dtheta = slam_pose.yaw - gt_pose.yaw
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        self.rt_align_rot = np.array([[c, -s], [s, c]], dtype=float)
        slam_xy = np.array([slam_pose.x, slam_pose.y], dtype=float)
        gt_xy = np.array([gt_pose.x, gt_pose.y], dtype=float)
        self.rt_align_trans = slam_xy - self.rt_align_rot @ gt_xy
        self.rt_align_ready = True

    def align_gt_pose(self, pose: Pose2DStamped) -> Pose2DStamped:
        xy = np.array([[pose.x, pose.y]], dtype=float)
        xy_aligned = (self.rt_align_rot @ xy.T).T + self.rt_align_trans
        yaw_aligned = self.normalize_angle(pose.yaw + math.atan2(self.rt_align_rot[1, 0], self.rt_align_rot[0, 0]))
        return Pose2DStamped(t=pose.t, x=float(xy_aligned[0, 0]), y=float(xy_aligned[0, 1]), yaw=yaw_aligned)

    def on_innovation(self, msg: Float64MultiArray) -> None:
        data = msg.data
        if data is None or len(data) < 3:
            return
        try:
            t = float(data[0])
            update_type = int(round(float(data[1])))
            dim = int(round(float(data[2])))
        except (ValueError, TypeError):
            return
        if dim <= 0:
            return
        expected = 3 + dim + dim * dim
        if len(data) < expected:
            return
        innov = np.array(data[3:3 + dim], dtype=float).reshape(dim, 1)
        s_flat = np.array(data[3 + dim:expected], dtype=float)
        S = s_flat.reshape(dim, dim)
        if not np.all(np.isfinite(innov)) or not np.all(np.isfinite(S)):
            return
        S = 0.5 * (S + S.T)
        inv_S = self.safe_inv_covariance(S)
        if inv_S is None:
            return
        nis = float(innov.T @ inv_S @ innov)
        self.last_innov_time = t
        self.last_innov_nis = nis
        self.last_innov_type = update_type

    def extract_pose_covariance(self, msg: Odometry) -> Optional[np.ndarray]:
        cov = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
        if not np.all(np.isfinite(cov)):
            return None
        if np.allclose(cov, 0.0):
            return None
        idx = [0, 1, 5]
        P = cov[np.ix_(idx, idx)].astype(float)
        P = 0.5 * (P + P.T)
        if self.cov_eig_floor > 0.0:
            eigvals, eigvecs = np.linalg.eigh(P)
            if not np.all(np.isfinite(eigvals)):
                return None
            eigvals = np.maximum(eigvals, self.cov_eig_floor)
            P = eigvecs @ np.diag(eigvals) @ eigvecs.T
        return P

    @staticmethod
    def safe_inv_covariance(cov: np.ndarray) -> Optional[np.ndarray]:
        try:
            return np.linalg.inv(cov)
        except np.linalg.LinAlgError:
            return None

    def compute_metrics(self, slam_arr: np.ndarray, gt_arr: np.ndarray) -> dict:
        dx = slam_arr[:, 1] - gt_arr[:, 1]
        dy = slam_arr[:, 2] - gt_arr[:, 2]
        pos_err = np.sqrt(dx * dx + dy * dy)
        yaw_err = np.array([self.normalize_angle(a - b) for a, b in zip(slam_arr[:, 3], gt_arr[:, 3])])

        ate_rmse = float(np.sqrt(np.mean(pos_err ** 2)))
        mae = float(np.mean(np.abs(pos_err)))
        max_err = float(np.max(pos_err))
        yaw_rmse_deg = float(math.degrees(np.sqrt(np.mean(yaw_err ** 2))))

        rpe_t_rmse, rpe_yaw_rmse_deg = self.compute_rpe(slam_arr, gt_arr, self.rpe_delta_samples)

        duration = float(slam_arr[-1, 0] - slam_arr[0, 0])
        path_len_slam = self.path_length(slam_arr[:, 1], slam_arr[:, 2])
        path_len_gt = self.path_length(gt_arr[:, 1], gt_arr[:, 2])

        return {
            'samples': int(slam_arr.shape[0]),
            'duration_sec': duration,
            'path_length_slam_m': path_len_slam,
            'path_length_gt_m': path_len_gt,
            'ate_rmse_m': ate_rmse,
            'position_mae_m': mae,
            'position_max_m': max_err,
            'yaw_rmse_deg': yaw_rmse_deg,
            'rpe_trans_rmse_m': rpe_t_rmse,
            'rpe_yaw_rmse_deg': rpe_yaw_rmse_deg,
        }

    def compute_rpe(self, slam_arr: np.ndarray, gt_arr: np.ndarray, delta: int) -> Tuple[float, float]:
        if slam_arr.shape[0] <= delta:
            return float('nan'), float('nan')

        trans_err = []
        yaw_err = []
        for i in range(0, slam_arr.shape[0] - delta):
            j = i + delta

            s_dx = slam_arr[j, 1] - slam_arr[i, 1]
            s_dy = slam_arr[j, 2] - slam_arr[i, 2]
            g_dx = gt_arr[j, 1] - gt_arr[i, 1]
            g_dy = gt_arr[j, 2] - gt_arr[i, 2]

            trans_err.append(math.hypot(s_dx - g_dx, s_dy - g_dy))

            s_dyaw = self.normalize_angle(slam_arr[j, 3] - slam_arr[i, 3])
            g_dyaw = self.normalize_angle(gt_arr[j, 3] - gt_arr[i, 3])
            yaw_err.append(self.normalize_angle(s_dyaw - g_dyaw))

        trans_err = np.array(trans_err, dtype=float)
        yaw_err = np.array(yaw_err, dtype=float)
        return float(np.sqrt(np.mean(trans_err ** 2))), float(math.degrees(np.sqrt(np.mean(yaw_err ** 2))))

    def generate_plots(
        self,
        run_dir: str,
        slam_arr: np.ndarray,
        gt_arr: np.ndarray,
        metrics: dict,
        landmarks_xy: List[Tuple[float, float]],
    ) -> None:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        t = slam_arr[:, 0] - slam_arr[0, 0]
        dx = slam_arr[:, 1] - gt_arr[:, 1]
        dy = slam_arr[:, 2] - gt_arr[:, 2]
        pos_err = np.sqrt(dx * dx + dy * dy)
        yaw_err = np.array([math.degrees(self.normalize_angle(a - b)) for a, b in zip(slam_arr[:, 3], gt_arr[:, 3])])

        fig1, ax1 = plt.subplots(figsize=(10, 8))
        ax1.plot(gt_arr[:, 1], gt_arr[:, 2], 'k-', linewidth=2.0, label='Ground Truth')
        ax1.plot(slam_arr[:, 1], slam_arr[:, 2], 'b--', linewidth=1.7, label='EKF-SLAM')
        if landmarks_xy:
            lm = np.array(landmarks_xy, dtype=float)
            ax1.scatter(lm[:, 0], lm[:, 1], s=20, c='orange', marker='o', alpha=0.8, label='SLAM Landmarks')
        ax1.set_title('Trajectory and Landmark Map')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        fig1.tight_layout()
        fig1.savefig(os.path.join(run_dir, 'trajectory_landmarks.png'), dpi=180)
        plt.close(fig1)

        fig2, axes = plt.subplots(2, 2, figsize=(13, 9))
        axes[0, 0].plot(t, pos_err, color='crimson', linewidth=1.5)
        axes[0, 0].set_title('Position Error Over Time')
        axes[0, 0].set_xlabel('Time [s]')
        axes[0, 0].set_ylabel('Error [m]')
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].plot(t, yaw_err, color='purple', linewidth=1.4)
        axes[0, 1].set_title('Yaw Error Over Time')
        axes[0, 1].set_xlabel('Time [s]')
        axes[0, 1].set_ylabel('Error [deg]')
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].hist(pos_err, bins=40, color='teal', alpha=0.8)
        axes[1, 0].set_title('Position Error Histogram')
        axes[1, 0].set_xlabel('Error [m]')
        axes[1, 0].set_ylabel('Count')
        axes[1, 0].grid(True, alpha=0.3)

        summary = (
            f"samples: {metrics['samples']}\n"
            f"duration: {metrics['duration_sec']:.2f} s\n"
            f"ATE RMSE: {metrics['ate_rmse_m']:.4f} m\n"
            f"Pos MAE: {metrics['position_mae_m']:.4f} m\n"
            f"Pos Max: {metrics['position_max_m']:.4f} m\n"
            f"Yaw RMSE: {metrics['yaw_rmse_deg']:.3f} deg\n"
            f"RPE trans RMSE: {metrics['rpe_trans_rmse_m']:.4f} m\n"
            f"RPE yaw RMSE: {metrics['rpe_yaw_rmse_deg']:.3f} deg\n"
            f"SLAM path length: {metrics['path_length_slam_m']:.2f} m\n"
            f"GT path length: {metrics['path_length_gt_m']:.2f} m"
        )
        axes[1, 1].axis('off')
        axes[1, 1].text(0.02, 0.98, summary, va='top', ha='left', fontsize=11, family='monospace')

        fig2.suptitle('EKF-SLAM Quantitative Evaluation', fontsize=14)
        fig2.tight_layout(rect=[0, 0, 1, 0.97])
        fig2.savefig(os.path.join(run_dir, 'error_metrics.png'), dpi=180)
        plt.close(fig2)

    def save_metrics_json(self, run_dir: str, metrics: dict) -> None:
        path = os.path.join(run_dir, 'metrics.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(metrics, f, indent=2)

    def save_time_series_csv(self, run_dir: str, slam_arr: np.ndarray, gt_arr: np.ndarray) -> None:
        path = os.path.join(run_dir, 'timeseries.csv')
        with open(path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                't',
                'slam_x', 'slam_y', 'slam_yaw_rad',
                'gt_x', 'gt_y', 'gt_yaw_rad',
                'err_x', 'err_y', 'err_pos', 'err_yaw_rad',
            ])
            for i in range(slam_arr.shape[0]):
                err_x = float(slam_arr[i, 1] - gt_arr[i, 1])
                err_y = float(slam_arr[i, 2] - gt_arr[i, 2])
                err_pos = float(math.hypot(err_x, err_y))
                err_yaw = float(self.normalize_angle(slam_arr[i, 3] - gt_arr[i, 3]))
                writer.writerow([
                    float(slam_arr[i, 0]),
                    float(slam_arr[i, 1]), float(slam_arr[i, 2]), float(slam_arr[i, 3]),
                    float(gt_arr[i, 1]), float(gt_arr[i, 2]), float(gt_arr[i, 3]),
                    err_x, err_y, err_pos, err_yaw,
                ])

    @staticmethod
    def path_length(x: np.ndarray, y: np.ndarray) -> float:
        if x.size < 2:
            return 0.0
        dx = np.diff(x)
        dy = np.diff(y)
        return float(np.sum(np.sqrt(dx * dx + dy * dy)))

    @staticmethod
    def stamp_to_sec(sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) * 1e-9

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main() -> None:
    rclpy.init()
    node = SlamEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_realtime_csv()
        if node.auto_export_on_shutdown:
            ok, message = node.export_all()
            level = node.get_logger().info if ok else node.get_logger().warn
            level(message)
        if node.rt_plot_ready and node.rt_pl is not None:
            try:
                node.rt_pl.close('all')
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
