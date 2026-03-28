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

        self.slam_odom_topic = str(self.get_parameter('slam_odom_topic').value)
        self.gt_odom_topic = str(self.get_parameter('gt_odom_topic').value)
        self.landmark_topic = str(self.get_parameter('landmark_topic').value)
        self.output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        self.run_name = str(self.get_parameter('run_name').value)
        self.rpe_delta_samples = int(self.get_parameter('rpe_delta_samples').value)
        self.auto_export_on_shutdown = bool(self.get_parameter('auto_export_on_shutdown').value)

        self.slam_poses: List[Pose2DStamped] = []
        self.gt_poses: List[Pose2DStamped] = []
        self.landmarks_xy: List[Tuple[float, float]] = []

        self.create_subscription(Odometry, self.slam_odom_topic, self.on_slam_odom, 50)
        self.create_subscription(Odometry, self.gt_odom_topic, self.on_gt_odom, 50)
        self.create_subscription(MarkerArray, self.landmark_topic, self.on_landmarks, 10)
        self.export_srv = self.create_service(Trigger, '/slam_eval/export', self.on_export)


        self.get_logger().info('slam_evaluator active: collecting slam/gt trajectories and landmarks')

    def on_slam_odom(self, msg: Odometry) -> None:
        t = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.slam_poses.append(Pose2DStamped(
            t=t,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        ))

    def on_gt_odom(self, msg: Odometry) -> None:
        t = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        yaw = self.yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.gt_poses.append(Pose2DStamped(
            t=t,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw,
        ))

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
        if node.auto_export_on_shutdown:
            ok, message = node.export_all()
            level = node.get_logger().info if ok else node.get_logger().warn
            level(message)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
