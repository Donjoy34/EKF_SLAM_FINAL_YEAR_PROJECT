import math
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

import numpy as np
import rclpy
from eufs_msgs.msg import ConeArrayWithCovariance, WheelSpeeds, WheelSpeedsStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


@dataclass
class Pose2D:
    t: float
    x: float
    y: float
    yaw: float


class EkfSlam(Node):
    def __init__(self) -> None:
        super().__init__('ekf_slam')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cone_topic', '/cones')
        self.declare_parameter('wheel_topic', '/ros_can/wheel_speeds')
        self.declare_parameter('wheel_topic_is_stamped', True)

        self.declare_parameter('gt_odom_topic', '/ground_truth/odom')
        self.declare_parameter('gt_cones_topic', '/ground_truth/cones')
        self.declare_parameter('gt_track_topic', '/ground_truth/track')
        self.declare_parameter('odom_topic', '/ekf_slam/odom')
        self.declare_parameter('path_topic', '/ekf_slam/path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('enable_live_plot', True)
        self.declare_parameter('live_plot_period_sec', 0.2)
        self.declare_parameter('align_gt_plot', True)

        self.declare_parameter('enable_loop_closure', True)
        self.declare_parameter('loop_search_radius_m', 3.5)
        self.declare_parameter('loop_yaw_gate_deg', 35.0)
        self.declare_parameter('loop_cooldown_sec', 2.0)
        self.declare_parameter('loop_min_landmark_overlap', 3)
        self.declare_parameter('loop_pose_noise_xy', 0.12)
        self.declare_parameter('loop_pose_noise_yaw_deg', 4.0)
        self.declare_parameter('keyframe_add_dist_m', 1.0)
        self.declare_parameter('keyframe_add_yaw_deg', 12.0)
        self.declare_parameter('keyframe_landmark_radius_m', 8.0)

        self.declare_parameter('wheel_radius_m', 0.2525)
        self.declare_parameter('wheelbase_m', 1.6)
        self.declare_parameter('max_dt_sec', 0.1)
        self.declare_parameter('wheel_speed_timeout_sec', 0.25)
        self.declare_parameter('wheel_invalid_rpm_threshold', 900.0)
        self.declare_parameter('wheel_speed_deadband_mps', 0.08)

        self.declare_parameter('use_imu_accel_prediction', False)
        self.declare_parameter('use_imu_orientation_update', True)
        self.declare_parameter('imu_yaw_meas_noise_deg', 2.5)

        self.declare_parameter('process_accel_noise', 1.2)
        self.declare_parameter('process_gyro_noise', 0.5)
        self.declare_parameter('process_gyro_bias_rw', 0.02)
        self.declare_parameter('wheel_speed_noise', 0.35)

        self.declare_parameter('cone_meas_noise_x', 0.35)
        self.declare_parameter('cone_meas_noise_y', 0.35)
        self.declare_parameter('association_gate_chi2', 6.0)
        self.declare_parameter('innovation_gate_chi2', 7.0)
        self.declare_parameter('obs_max_range_m', 20.0)
        self.declare_parameter('max_observations_per_scan', 120)
        self.declare_parameter('max_landmarks', 600)
        self.declare_parameter('landmark_min_init_range_m', 1.0)

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.cone_topic = str(self.get_parameter('cone_topic').value)
        self.wheel_topic = str(self.get_parameter('wheel_topic').value)
        self.wheel_topic_is_stamped = bool(self.get_parameter('wheel_topic_is_stamped').value)

        self.gt_odom_topic = str(self.get_parameter('gt_odom_topic').value)
        self.gt_cones_topic = str(self.get_parameter('gt_cones_topic').value)
        self.gt_track_topic = str(self.get_parameter('gt_track_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.enable_live_plot = bool(self.get_parameter('enable_live_plot').value)
        self.live_plot_period_sec = float(self.get_parameter('live_plot_period_sec').value)
        self.align_gt_plot = bool(self.get_parameter('align_gt_plot').value)

        self.enable_loop_closure = bool(self.get_parameter('enable_loop_closure').value)
        self.loop_search_radius_m = float(self.get_parameter('loop_search_radius_m').value)
        self.loop_yaw_gate_rad = math.radians(float(self.get_parameter('loop_yaw_gate_deg').value))
        self.loop_cooldown_sec = float(self.get_parameter('loop_cooldown_sec').value)
        self.loop_min_landmark_overlap = int(self.get_parameter('loop_min_landmark_overlap').value)
        self.loop_pose_noise_xy = float(self.get_parameter('loop_pose_noise_xy').value)
        self.loop_pose_noise_yaw_rad = math.radians(float(self.get_parameter('loop_pose_noise_yaw_deg').value))
        self.keyframe_add_dist_m = float(self.get_parameter('keyframe_add_dist_m').value)
        self.keyframe_add_yaw_rad = math.radians(float(self.get_parameter('keyframe_add_yaw_deg').value))
        self.keyframe_landmark_radius_m = float(self.get_parameter('keyframe_landmark_radius_m').value)

        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheelbase_m = float(self.get_parameter('wheelbase_m').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.wheel_speed_timeout_sec = float(self.get_parameter('wheel_speed_timeout_sec').value)
        self.wheel_invalid_rpm_threshold = float(self.get_parameter('wheel_invalid_rpm_threshold').value)
        self.wheel_speed_deadband_mps = float(self.get_parameter('wheel_speed_deadband_mps').value)

        self.use_imu_accel_prediction = bool(self.get_parameter('use_imu_accel_prediction').value)
        self.use_imu_orientation_update = bool(self.get_parameter('use_imu_orientation_update').value)
        self.imu_yaw_meas_noise_rad = math.radians(float(self.get_parameter('imu_yaw_meas_noise_deg').value))

        self.process_accel_noise = float(self.get_parameter('process_accel_noise').value)
        self.process_gyro_noise = float(self.get_parameter('process_gyro_noise').value)
        self.process_gyro_bias_rw = float(self.get_parameter('process_gyro_bias_rw').value)
        self.wheel_speed_noise = float(self.get_parameter('wheel_speed_noise').value)

        self.cone_meas_noise_x = float(self.get_parameter('cone_meas_noise_x').value)
        self.cone_meas_noise_y = float(self.get_parameter('cone_meas_noise_y').value)
        self.association_gate_chi2 = float(self.get_parameter('association_gate_chi2').value)
        self.innovation_gate_chi2 = float(self.get_parameter('innovation_gate_chi2').value)
        self.obs_max_range_m = float(self.get_parameter('obs_max_range_m').value)
        self.max_observations_per_scan = int(self.get_parameter('max_observations_per_scan').value)
        self.max_landmarks = int(self.get_parameter('max_landmarks').value)
        self.landmark_min_init_range_m = float(self.get_parameter('landmark_min_init_range_m').value)

        self.base_dim = 5
        self.x = np.zeros((self.base_dim, 1), dtype=float)
        self.P = np.diag([1.0, 1.0, 0.5, 1.0, 0.1]).astype(float)

        self.last_imu_time: Optional[float] = None
        self.last_wheel_time: Optional[float] = None
        self.last_wheel_speed_mps = 0.0
        self.last_steering_rad = 0.0
        self.filter_ready = False
        self.last_loop_time = -1e9
        self.keyframes: List[dict] = []

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.estimated_path: List[Pose2D] = []
        self.gt_path: List[Pose2D] = []
        self.gt_landmarks_last: List[Tuple[float, float]] = []
        self.gt_landmarks_static: List[Tuple[float, float]] = []

        self.plt = None
        self.live_plot_ready = False
        self.fig = None
        self.ax = None
        self.est_line = None
        self.gt_line = None
        self.pred_landmark_scatter = None
        self.gt_landmark_scatter = None
        self.est_car_line = None
        self.gt_car_line = None
        self.plot_align_ready = False
        self.plot_rot = np.eye(2, dtype=float)
        self.plot_trans = np.zeros((2,), dtype=float)

        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(ConeArrayWithCovariance, self.cone_topic, self.on_cones, qos_profile_sensor_data)
        if self.wheel_topic_is_stamped:
            self.create_subscription(WheelSpeedsStamped, self.wheel_topic, self.on_wheel_stamped, qos_profile_sensor_data)
        else:
            self.create_subscription(WheelSpeeds, self.wheel_topic, self.on_wheel, qos_profile_sensor_data)

        self.create_subscription(Odometry, self.gt_odom_topic, self.on_gt_odom, 50)
        self.create_subscription(ConeArrayWithCovariance, self.gt_cones_topic, self.on_gt_cones, 20)
        self.create_subscription(ConeArrayWithCovariance, self.gt_track_topic, self.on_gt_track, 10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        if self.enable_live_plot:
            self.setup_live_plot()
            if self.live_plot_ready:
                self.create_timer(max(0.05, self.live_plot_period_sec), self.update_live_plot)

        self.get_logger().info('ekf_slam started (simplified EKF-SLAM)')

    def setup_live_plot(self) -> None:
        try:
            import matplotlib.pyplot as plt

            plt.ion()
            self.plt = plt
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            self.est_line, = self.ax.plot([], [], 'b-', linewidth=1.8, label='EKF Path')
            self.gt_line, = self.ax.plot([], [], 'k--', linewidth=1.6, label='GT Path')
            self.pred_landmark_scatter = self.ax.scatter([], [], s=22, c='orange', alpha=0.9, label='Predicted Landmarks')
            self.gt_landmark_scatter = self.ax.scatter([], [], s=12, c='green', alpha=0.6, label='GT Landmarks')
            self.est_car_line, = self.ax.plot([], [], 'r-', linewidth=2.0, label='EKF Car')
            self.gt_car_line, = self.ax.plot([], [], color='dimgray', linewidth=1.8, label='GT Car')
            self.ax.set_title('EKF-SLAM Live 2D Map')
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.grid(True, alpha=0.3)
            self.ax.legend(loc='best')

            self.live_plot_ready = True
            self.plt.show(block=False)
        except Exception as exc:
            self.live_plot_ready = False
            self.get_logger().warn(f'live matplotlib plotting disabled: {exc}')

    def update_live_plot(self) -> None:
        if not self.live_plot_ready or self.ax is None or self.fig is None or self.plt is None:
            return
        if len(self.estimated_path) < 2:
            return

        if self.align_gt_plot and self.gt_path and not self.plot_align_ready:
            self.init_plot_alignment()

        est_xy = np.array([(p.x, p.y) for p in self.estimated_path], dtype=float)
        gt_xy = np.array([(p.x, p.y) for p in self.gt_path], dtype=float) if self.gt_path else np.zeros((0, 2), dtype=float)
        pred_landmarks = self.get_predicted_landmarks()

        if self.align_gt_plot and self.plot_align_ready:
            gt_xy = self.align_points(gt_xy)
            pred_gt_landmarks = (
                self.align_points(np.array(self.gt_landmarks_static, dtype=float))
                if self.gt_landmarks_static
                else None
            )
        else:
            pred_gt_landmarks = np.array(self.gt_landmarks_static, dtype=float) if self.gt_landmarks_static else None

        self.est_line.set_data(est_xy[:, 0], est_xy[:, 1])
        if gt_xy.shape[0] > 1:
            self.gt_line.set_data(gt_xy[:, 0], gt_xy[:, 1])
        else:
            self.gt_line.set_data([], [])

        if pred_landmarks.shape[0] > 0:
            self.pred_landmark_scatter.set_offsets(pred_landmarks)
        else:
            self.pred_landmark_scatter.set_offsets(np.empty((0, 2), dtype=float))

        if pred_gt_landmarks is not None and pred_gt_landmarks.size > 0:
            self.gt_landmark_scatter.set_offsets(pred_gt_landmarks)
        else:
            self.gt_landmark_scatter.set_offsets(np.empty((0, 2), dtype=float))

        est_pose = self.estimated_path[-1]
        est_tri = self.car_triangle(est_pose.x, est_pose.y, est_pose.yaw)
        self.est_car_line.set_data(est_tri[:, 0], est_tri[:, 1])

        if self.gt_path:
            gt_pose = self.gt_path[-1]
            if self.align_gt_plot and self.plot_align_ready:
                gt_pose = self.align_pose(gt_pose)
            gt_tri = self.car_triangle(gt_pose.x, gt_pose.y, gt_pose.yaw)
            self.gt_car_line.set_data(gt_tri[:, 0], gt_tri[:, 1])
        else:
            self.gt_car_line.set_data([], [])

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.set_aspect('equal', adjustable='box')
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.plt.pause(0.001)

    def state_dim(self) -> int:
        return self.x.shape[0]

    def landmark_count(self) -> int:
        return max(0, (self.state_dim() - self.base_dim) // 2)

    def landmark_slice(self, landmark_index: int) -> slice:
        start = self.base_dim + 2 * landmark_index
        return slice(start, start + 2)

    def on_imu(self, msg: Imu) -> None:
        stamp = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = stamp
            self.x[2, 0] = self.yaw_from_quaternion(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
            self.filter_ready = True
            self.publish_outputs(stamp)
            return

        dt = stamp - self.last_imu_time
        self.last_imu_time = stamp
        if dt <= 1e-6:
            return
        dt = min(dt, self.max_dt_sec)

        yaw = float(self.x[2, 0])
        v = float(self.x[3, 0])
        b_g = float(self.x[4, 0])
        omega_z = float(msg.angular_velocity.z) - b_g
        accel_x = float(msg.linear_acceleration.x)

        wheel_fresh = False
        if self.last_wheel_time is not None:
            wheel_fresh = abs(stamp - self.last_wheel_time) <= self.wheel_speed_timeout_sec
        v_wheel = self.last_wheel_speed_mps if wheel_fresh else v

        if self.use_imu_accel_prediction:
            v_pred = v + accel_x * dt
        else:
            v_pred = v_wheel

        v_used = v_wheel if wheel_fresh else v_pred

        c_yaw = math.cos(yaw)
        s_yaw = math.sin(yaw)
        if self.use_imu_accel_prediction:
            px_new = float(self.x[0, 0]) + v_used * c_yaw * dt + 0.5 * accel_x * c_yaw * dt * dt
            py_new = float(self.x[1, 0]) + v_used * s_yaw * dt + 0.5 * accel_x * s_yaw * dt * dt
        else:
            px_new = float(self.x[0, 0]) + v_used * c_yaw * dt
            py_new = float(self.x[1, 0]) + v_used * s_yaw * dt
        yaw_new = self.normalize_angle(yaw + omega_z * dt)
        v_new = v_pred

        n = self.state_dim()
        F = np.eye(n, dtype=float)
        F[0, 2] = -v_used * s_yaw * dt
        F[1, 2] = v_used * c_yaw * dt
        F[0, 3] = c_yaw * dt
        F[1, 3] = s_yaw * dt
        F[2, 4] = -dt

        Q = np.zeros((n, n), dtype=float)
        accel_std = max(0.1, self.process_accel_noise)
        Q[0, 0] = (0.5 * accel_std * dt * dt) ** 2
        Q[1, 1] = Q[0, 0]
        Q[3, 3] = (accel_std * dt) ** 2
        gyro_std = max(0.1, self.process_gyro_noise)
        Q[2, 2] = (gyro_std * dt) ** 2
        Q[4, 4] = (self.process_gyro_bias_rw * math.sqrt(dt)) ** 2

        self.x[0, 0] = px_new
        self.x[1, 0] = py_new
        self.x[2, 0] = yaw_new
        self.x[3, 0] = v_new

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

        if self.use_imu_orientation_update:
            self.ekf_update_yaw_from_imu(msg)

        self.publish_outputs(stamp)

    def on_wheel_stamped(self, msg: WheelSpeedsStamped) -> None:
        stamp = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp <= 0.0:
            stamp = None
        self.on_wheel(msg.speeds, stamp)

    def on_wheel(self, msg: WheelSpeeds, stamp: Optional[float] = None) -> None:
        if not self.filter_ready:
            return

        if stamp is None:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self.last_steering_rad = float(msg.steering)

        wheel_rpms = [
            abs(float(msg.lf_speed)),
            abs(float(msg.rf_speed)),
            abs(float(msg.lb_speed)),
            abs(float(msg.rb_speed)),
        ]
        valid = [rpm for rpm in wheel_rpms if math.isfinite(rpm) and rpm < self.wheel_invalid_rpm_threshold]
        if not valid:
            return

        wheel_rpm = float(np.median(np.array(valid, dtype=float)))
        speed_mps = (wheel_rpm / 60.0) * (2.0 * math.pi * self.wheel_radius_m)
        if abs(speed_mps) < self.wheel_speed_deadband_mps:
            speed_mps = 0.0

        self.last_wheel_speed_mps = speed_mps
        self.last_wheel_time = stamp
        self.ekf_update_speed(speed_mps)

    def ekf_update_speed(self, speed_measurement: float) -> None:
        n = self.state_dim()
        H = np.zeros((1, n), dtype=float)
        H[0, 3] = 1.0
        R = np.array([[max(1e-6, self.wheel_speed_noise ** 2)]], dtype=float)

        z = np.array([[speed_measurement]], dtype=float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ S_inv

        self.x = self.x + K @ y
        I = np.eye(n)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def ekf_update_yaw_from_imu(self, msg: Imu) -> None:
        yaw_meas = self.yaw_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        n = self.state_dim()
        H = np.zeros((1, n), dtype=float)
        H[0, 2] = 1.0
        R = np.array([[self.imu_yaw_meas_noise_rad ** 2]], dtype=float)

        z = np.array([[yaw_meas]], dtype=float)
        h = np.array([[float(self.x[2, 0])]], dtype=float)
        y = z - h
        y[0, 0] = self.normalize_angle(float(y[0, 0]))

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        self.x[2, 0] = self.normalize_angle(float(self.x[2, 0]))

        I = np.eye(n)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def on_cones(self, msg: ConeArrayWithCovariance) -> None:
        if not self.filter_ready:
            return

        observations = self.build_cone_observations(msg)
        if not observations:
            return

        used_landmarks: Set[int] = set()
        processed = 0
        matched_indices: List[int] = []
        for zx, zy, R in observations:
            if processed >= self.max_observations_per_scan:
                break
            obs_range = math.hypot(zx, zy)
            if obs_range > self.obs_max_range_m:
                continue

            lm_index = self.associate_landmark(zx, zy, R, used_landmarks)
            if lm_index is None:
                if self.landmark_count() < self.max_landmarks and obs_range >= self.landmark_min_init_range_m:
                    self.augment_landmark(zx, zy, R)
            else:
                self.ekf_update_landmark(lm_index, zx, zy, R)
                used_landmarks.add(lm_index)
                matched_indices.append(lm_index)
            processed += 1

        if self.enable_loop_closure:
            self.update_loop_closure(matched_indices)

    def on_gt_odom(self, msg: Odometry) -> None:
        t = self.stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.gt_path.append(Pose2D(
            t=t,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=self.yaw_from_quaternion(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
        ))

    def init_plot_alignment(self) -> None:
        if not self.estimated_path or not self.gt_path:
            return
        est0 = self.estimated_path[0]
        gt0 = self.gt_path[0]
        dtheta = est0.yaw - gt0.yaw
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        self.plot_rot = np.array([[c, -s], [s, c]], dtype=float)
        est0_vec = np.array([est0.x, est0.y], dtype=float)
        gt0_vec = np.array([gt0.x, gt0.y], dtype=float)
        self.plot_trans = est0_vec - self.plot_rot @ gt0_vec
        self.plot_align_ready = True

    def align_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0 or not self.plot_align_ready:
            return points
        return (self.plot_rot @ points.T).T + self.plot_trans

    def align_pose(self, pose: Pose2D) -> Pose2D:
        if not self.plot_align_ready:
            return pose
        xy = np.array([[pose.x, pose.y]], dtype=float)
        xy_aligned = self.align_points(xy)
        yaw_aligned = self.normalize_angle(pose.yaw + math.atan2(self.plot_rot[1, 0], self.plot_rot[0, 0]))
        return Pose2D(t=pose.t, x=float(xy_aligned[0, 0]), y=float(xy_aligned[0, 1]), yaw=yaw_aligned)

    def on_gt_cones(self, msg: ConeArrayWithCovariance) -> None:
        gt = []
        for cone in msg.blue_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.yellow_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.orange_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.big_orange_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        if gt:
            self.gt_landmarks_last = gt
            # Do not accumulate cones from a sensor view; keep as a live snapshot.

    def on_gt_track(self, msg: ConeArrayWithCovariance) -> None:
        # Track topic typically contains the full static map of cones.
        gt = []
        for cone in msg.blue_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.yellow_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.orange_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.big_orange_cones:
            gt.append((float(cone.point.x), float(cone.point.y)))
        if gt:
            self.gt_landmarks_static = gt
            self.gt_landmarks_last = gt

    def merge_gt_landmarks(self, gt: List[Tuple[float, float]], merge_dist: float = 0.6) -> None:
        if not self.gt_landmarks_static:
            self.gt_landmarks_static = list(gt)
            return

        merge_dist2 = merge_dist * merge_dist
        for gx, gy in gt:
            merged = False
            for idx, (sx, sy) in enumerate(self.gt_landmarks_static):
                dx = gx - sx
                dy = gy - sy
                if dx * dx + dy * dy <= merge_dist2:
                    self.gt_landmarks_static[idx] = (0.5 * (sx + gx), 0.5 * (sy + gy))
                    merged = True
                    break
            if not merged:
                self.gt_landmarks_static.append((gx, gy))

    def get_nearby_landmark_ids(self, x: float, y: float, radius: float) -> Set[int]:
        ids: Set[int] = set()
        r2 = radius * radius
        for idx in range(self.landmark_count()):
            sl = self.landmark_slice(idx)
            dx = float(self.x[sl.start, 0]) - x
            dy = float(self.x[sl.start + 1, 0]) - y
            if dx * dx + dy * dy <= r2:
                ids.add(idx)
        return ids

    def maybe_add_keyframe(self, now_sec: float) -> None:
        px = float(self.x[0, 0])
        py = float(self.x[1, 0])
        yaw = float(self.x[2, 0])
        nearby = self.get_nearby_landmark_ids(px, py, self.keyframe_landmark_radius_m)

        if not self.keyframes:
            self.keyframes.append({'x': px, 'y': py, 'yaw': yaw, 'time': now_sec, 'landmarks': nearby})
            return

        last = self.keyframes[-1]
        dist = math.hypot(px - last['x'], py - last['y'])
        dyaw = abs(self.normalize_angle(yaw - last['yaw']))
        if dist >= self.keyframe_add_dist_m or dyaw >= self.keyframe_add_yaw_rad:
            self.keyframes.append({'x': px, 'y': py, 'yaw': yaw, 'time': now_sec, 'landmarks': nearby})

    def update_loop_closure(self, matched_landmark_indices: List[int]) -> None:
        if not self.filter_ready:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.maybe_add_keyframe(now_sec)

        if len(self.keyframes) < 6:
            return
        if now_sec - self.last_loop_time < self.loop_cooldown_sec:
            return

        px = float(self.x[0, 0])
        py = float(self.x[1, 0])
        yaw = float(self.x[2, 0])
        matched_set = set(matched_landmark_indices)

        best = None
        best_score = float('inf')
        for keyframe in self.keyframes[:-3]:
            dist = math.hypot(px - keyframe['x'], py - keyframe['y'])
            if dist > self.loop_search_radius_m:
                continue
            dyaw = abs(self.normalize_angle(yaw - keyframe['yaw']))
            if dyaw > self.loop_yaw_gate_rad:
                continue
            overlap = len(matched_set.intersection(keyframe['landmarks']))
            if overlap < self.loop_min_landmark_overlap:
                continue

            score = dist + 0.2 * dyaw
            if score < best_score:
                best_score = score
                best = keyframe

        if best is None:
            return

        self.apply_loop_pose_update(best['x'], best['y'], best['yaw'])
        self.last_loop_time = now_sec

    def apply_loop_pose_update(self, x_ref: float, y_ref: float, yaw_ref: float) -> None:
        n = self.state_dim()
        H = np.zeros((3, n), dtype=float)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        z = np.array([[x_ref], [y_ref], [yaw_ref]], dtype=float)
        h = np.array([[float(self.x[0, 0])], [float(self.x[1, 0])], [float(self.x[2, 0])]], dtype=float)
        y = z - h
        y[2, 0] = self.normalize_angle(float(y[2, 0]))

        R = np.diag([
            self.loop_pose_noise_xy ** 2,
            self.loop_pose_noise_xy ** 2,
            self.loop_pose_noise_yaw_rad ** 2,
        ])

        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.x = self.x + K @ y
        self.x[2, 0] = self.normalize_angle(float(self.x[2, 0]))

        I = np.eye(n)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def build_cone_observations(self, msg: ConeArrayWithCovariance) -> List[Tuple[float, float, np.ndarray]]:
        observations: List[Tuple[float, float, np.ndarray]] = []

        def append_group(cones) -> None:
            for cone in cones:
                zx = float(cone.point.x)
                zy = float(cone.point.y)
                c = cone.covariance
                r_x = max(self.cone_meas_noise_x ** 2, float(c[0]) if len(c) > 0 else self.cone_meas_noise_x ** 2)
                r_y = max(self.cone_meas_noise_y ** 2, float(c[3]) if len(c) > 3 else self.cone_meas_noise_y ** 2)
                R = np.diag([r_x, r_y])
                observations.append((zx, zy, R))

        append_group(msg.blue_cones)
        append_group(msg.yellow_cones)
        append_group(msg.orange_cones)
        append_group(msg.big_orange_cones)
        return observations

    def associate_landmark(
        self,
        zx: float,
        zy: float,
        R: np.ndarray,
        used_landmarks: Optional[Set[int]] = None,
    ) -> Optional[int]:
        landmark_count = self.landmark_count()
        if landmark_count == 0:
            return None

        best_idx = None
        best_d2 = float('inf')
        z = np.array([[zx], [zy]], dtype=float)

        for idx in range(landmark_count):
            if used_landmarks is not None and idx in used_landmarks:
                continue
            zhat, H = self.predict_landmark_measurement(idx)
            innovation = z - zhat
            S = H @ self.P @ H.T + R
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            d2 = float(innovation.T @ S_inv @ innovation)
            if d2 < best_d2:
                best_d2 = d2
                best_idx = idx

        if best_idx is None or best_d2 > self.association_gate_chi2:
            return None
        return best_idx

    def predict_landmark_measurement(self, landmark_index: int) -> Tuple[np.ndarray, np.ndarray]:
        n = self.state_dim()
        px = float(self.x[0, 0])
        py = float(self.x[1, 0])
        yaw = float(self.x[2, 0])
        c = math.cos(yaw)
        s = math.sin(yaw)

        sl = self.landmark_slice(landmark_index)
        lx = float(self.x[sl.start, 0])
        ly = float(self.x[sl.start + 1, 0])
        dx = lx - px
        dy = ly - py

        zhat = np.array([[c * dx + s * dy], [-s * dx + c * dy]], dtype=float)
        H = np.zeros((2, n), dtype=float)
        H[0, 0] = -c
        H[0, 1] = -s
        H[1, 0] = s
        H[1, 1] = -c
        H[0, 2] = -s * dx + c * dy
        H[1, 2] = -c * dx - s * dy
        H[0, sl.start] = c
        H[0, sl.start + 1] = s
        H[1, sl.start] = -s
        H[1, sl.start + 1] = c
        return zhat, H

    def ekf_update_landmark(self, landmark_index: int, zx: float, zy: float, R: np.ndarray) -> None:
        z = np.array([[zx], [zy]], dtype=float)
        zhat, H = self.predict_landmark_measurement(landmark_index)
        innovation = z - zhat
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        d2 = float(innovation.T @ S_inv @ innovation)
        if d2 > self.innovation_gate_chi2:
            return

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        self.x[2, 0] = self.normalize_angle(float(self.x[2, 0]))

        n = self.state_dim()
        I = np.eye(n)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def augment_landmark(self, zx: float, zy: float, R: np.ndarray) -> None:
        yaw = float(self.x[2, 0])
        c = math.cos(yaw)
        s = math.sin(yaw)
        px = float(self.x[0, 0])
        py = float(self.x[1, 0])

        lx = px + c * zx - s * zy
        ly = py + s * zx + c * zy

        n_old = self.state_dim()
        x_new = np.zeros((n_old + 2, 1), dtype=float)
        x_new[:n_old, 0] = self.x[:, 0]
        x_new[n_old, 0] = lx
        x_new[n_old + 1, 0] = ly

        P_new = np.zeros((n_old + 2, n_old + 2), dtype=float)
        P_new[:n_old, :n_old] = self.P

        Jxr = np.array([
            [1.0, 0.0, -s * zx - c * zy],
            [0.0, 1.0, c * zx - s * zy],
        ], dtype=float)
        Jz = np.array([[c, -s], [s, c]], dtype=float)

        P_rr = self.P[0:3, 0:3]
        P_rX = self.P[0:3, :]
        P_lmX = Jxr @ P_rX
        P_ll = Jxr @ P_rr @ Jxr.T + Jz @ R @ Jz.T

        P_new[n_old:n_old + 2, :n_old] = P_lmX
        P_new[:n_old, n_old:n_old + 2] = P_lmX.T
        P_new[n_old:n_old + 2, n_old:n_old + 2] = P_ll

        self.x = x_new
        self.P = 0.5 * (P_new + P_new.T)

    def publish_outputs(self, stamp_sec: float) -> None:
        sec = int(stamp_sec)
        nanosec = int((stamp_sec - sec) * 1e9)

        yaw = float(self.x[2, 0])
        qx, qy, qz, qw = self.quaternion_from_yaw(yaw)

        odom = Odometry()
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x[0, 0])
        odom.pose.pose.position.y = float(self.x[1, 0])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.x[3, 0])
        self.odom_pub.publish(odom)

        pose = Pose2D(t=stamp_sec, x=float(self.x[0, 0]), y=float(self.x[1, 0]), yaw=float(self.x[2, 0]))
        self.estimated_path.append(pose)

        msg = PoseStamped()
        msg.header = odom.header
        msg.pose = odom.pose.pose
        self.path_msg.header.stamp = odom.header.stamp
        self.path_msg.poses.append(msg)
        if len(self.path_msg.poses) > 4000:
            self.path_msg.poses = self.path_msg.poses[-4000:]
        self.path_pub.publish(self.path_msg)

    def get_predicted_landmarks(self) -> np.ndarray:
        count = self.landmark_count()
        if count <= 0:
            return np.zeros((0, 2), dtype=float)
        points = []
        for idx in range(count):
            sl = self.landmark_slice(idx)
            points.append((float(self.x[sl.start, 0]), float(self.x[sl.start + 1, 0])))
        if not points:
            return np.zeros((0, 2), dtype=float)
        return np.array(points, dtype=float)

    @staticmethod
    def car_triangle(x: float, y: float, yaw: float, length: float = 1.4, width: float = 0.9) -> np.ndarray:
        c = math.cos(yaw)
        s = math.sin(yaw)
        fx, fy = c, s
        lx, ly = -s, c

        nose = (x + 0.7 * length * fx, y + 0.7 * length * fy)
        rear_c = (x - 0.4 * length * fx, y - 0.4 * length * fy)
        rear_l = (rear_c[0] + 0.5 * width * lx, rear_c[1] + 0.5 * width * ly)
        rear_r = (rear_c[0] - 0.5 * width * lx, rear_c[1] - 0.5 * width * ly)
        return np.array([nose, rear_l, rear_r, nose], dtype=float)

    @staticmethod
    def stamp_to_sec(sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) * 1e-9

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)


def main() -> None:
    rclpy.init()
    node = EkfSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.live_plot_ready and node.plt is not None:
            try:
                node.plt.close('all')
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
