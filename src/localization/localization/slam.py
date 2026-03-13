import math
import json
import os
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from eufs_msgs.msg import ConeArrayWithCovariance, WheelSpeeds, WheelSpeedsStamped


class Slam(Node):
    def __init__(self) -> None:
        super().__init__('slam')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cone_topic', '/cones')
        self.declare_parameter('use_cone_updates', True)
        self.declare_parameter('state_topic', '/ros_can/state_str')
        self.declare_parameter('wheel_topic', '/ros_can/wheel_speeds')
        self.declare_parameter('wheel_topic_is_stamped', True)
        self.declare_parameter('odom_topic', '/slam/odom')
        self.declare_parameter('path_topic', '/slam/path')
        self.declare_parameter('landmark_topic', '/slam/landmarks')
        self.declare_parameter('occupancy_topic', '/slam/occupancy')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('wheel_radius_m', 0.2525)
        self.declare_parameter('max_dt_sec', 0.1)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('path_max_length', 4000)
        self.declare_parameter('init_cov_pos', 1.0)
        self.declare_parameter('init_cov_yaw', 0.5)
        self.declare_parameter('init_cov_vel', 1.0)
        self.declare_parameter('init_cov_gyro_bias', 0.1)
        self.declare_parameter('process_accel_noise', 1.2)
        self.declare_parameter('process_gyro_noise', 0.5)
        self.declare_parameter('process_gyro_bias_rw', 0.02)
        self.declare_parameter('wheel_speed_noise', 0.35)
        self.declare_parameter('cone_meas_noise_x', 0.35)
        self.declare_parameter('cone_meas_noise_y', 0.35)
        self.declare_parameter('association_gate_chi2', 6.0)
        self.declare_parameter('obs_max_range_m', 20.0)
        self.declare_parameter('max_observations_per_scan', 120)
        self.declare_parameter('max_landmarks', 600)
        self.declare_parameter('publish_landmark_markers', True)
        self.declare_parameter('landmark_marker_scale', 0.28)
        self.declare_parameter('map_file_path', '')
        self.declare_parameter('auto_save_map_on_shutdown', False)
        self.declare_parameter('loaded_landmark_std', 1.0)
        self.declare_parameter('occupancy_resolution_m', 0.25)
        self.declare_parameter('occupancy_width_cells', 480)
        self.declare_parameter('occupancy_height_cells', 480)
        self.declare_parameter('occupancy_origin_x', -60.0)
        self.declare_parameter('occupancy_origin_y', -60.0)
        self.declare_parameter('occupancy_logodds_hit', 0.85)
        self.declare_parameter('occupancy_logodds_miss', -0.35)
        self.declare_parameter('occupancy_logodds_min', -4.0)
        self.declare_parameter('occupancy_logodds_max', 4.0)
        self.declare_parameter('occupancy_publish_period_sec', 0.5)
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

        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.cone_topic = str(self.get_parameter('cone_topic').value)
        self.use_cone_updates = bool(self.get_parameter('use_cone_updates').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.wheel_topic = str(self.get_parameter('wheel_topic').value)
        self.wheel_topic_is_stamped = bool(self.get_parameter('wheel_topic_is_stamped').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.landmark_topic = str(self.get_parameter('landmark_topic').value)
        self.occupancy_topic = str(self.get_parameter('occupancy_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.max_dt_sec = float(self.get_parameter('max_dt_sec').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.path_max_length = int(self.get_parameter('path_max_length').value)

        init_cov_pos = float(self.get_parameter('init_cov_pos').value)
        init_cov_yaw = float(self.get_parameter('init_cov_yaw').value)
        init_cov_vel = float(self.get_parameter('init_cov_vel').value)
        init_cov_gyro_bias = float(self.get_parameter('init_cov_gyro_bias').value)

        self.process_accel_noise = float(self.get_parameter('process_accel_noise').value)
        self.process_gyro_noise = float(self.get_parameter('process_gyro_noise').value)
        self.process_gyro_bias_rw = float(self.get_parameter('process_gyro_bias_rw').value)
        self.wheel_speed_noise = float(self.get_parameter('wheel_speed_noise').value)
        self.cone_meas_noise_x = float(self.get_parameter('cone_meas_noise_x').value)
        self.cone_meas_noise_y = float(self.get_parameter('cone_meas_noise_y').value)
        self.association_gate_chi2 = float(self.get_parameter('association_gate_chi2').value)
        self.obs_max_range_m = float(self.get_parameter('obs_max_range_m').value)
        self.max_observations_per_scan = int(self.get_parameter('max_observations_per_scan').value)
        self.max_landmarks = int(self.get_parameter('max_landmarks').value)
        self.publish_landmark_markers = bool(self.get_parameter('publish_landmark_markers').value)
        self.landmark_marker_scale = float(self.get_parameter('landmark_marker_scale').value)
        self.map_file_path = str(self.get_parameter('map_file_path').value)
        self.auto_save_map_on_shutdown = bool(self.get_parameter('auto_save_map_on_shutdown').value)
        self.loaded_landmark_std = float(self.get_parameter('loaded_landmark_std').value)
        self.occupancy_resolution_m = float(self.get_parameter('occupancy_resolution_m').value)
        self.occupancy_width_cells = int(self.get_parameter('occupancy_width_cells').value)
        self.occupancy_height_cells = int(self.get_parameter('occupancy_height_cells').value)
        self.occupancy_origin_x = float(self.get_parameter('occupancy_origin_x').value)
        self.occupancy_origin_y = float(self.get_parameter('occupancy_origin_y').value)
        self.occupancy_logodds_hit = float(self.get_parameter('occupancy_logodds_hit').value)
        self.occupancy_logodds_miss = float(self.get_parameter('occupancy_logodds_miss').value)
        self.occupancy_logodds_min = float(self.get_parameter('occupancy_logodds_min').value)
        self.occupancy_logodds_max = float(self.get_parameter('occupancy_logodds_max').value)
        self.occupancy_publish_period_sec = float(self.get_parameter('occupancy_publish_period_sec').value)
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

        self.base_dim = 5
        self.base_P0 = np.diag([
            init_cov_pos ** 2,
            init_cov_pos ** 2,
            init_cov_yaw ** 2,
            init_cov_vel ** 2,
            init_cov_gyro_bias ** 2,
        ])
        self.x = np.zeros((self.base_dim, 1), dtype=float)
        self.P = self.base_P0.copy()

        self.last_imu_time: Optional[float] = None
        self.last_update_time: Optional[float] = None
        self.motion_enabled = False
        self.filter_ready = False

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.last_landmark_marker_count = 0
        self.occupancy_logodds = np.zeros((self.occupancy_height_cells, self.occupancy_width_cells), dtype=float)
        self.last_loop_time = -1e9
        self.keyframes = []

        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, self.state_topic, self.on_state_str, 10)
        if self.use_cone_updates:
            self.create_subscription(ConeArrayWithCovariance, self.cone_topic, self.on_cones, qos_profile_sensor_data)
        if self.wheel_topic_is_stamped:
            self.create_subscription(WheelSpeedsStamped, self.wheel_topic, self.on_wheel_stamped, qos_profile_sensor_data)
        else:
            self.create_subscription(WheelSpeeds, self.wheel_topic, self.on_wheel, qos_profile_sensor_data)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.landmark_pub = self.create_publisher(MarkerArray, self.landmark_topic, 10)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, self.occupancy_topic, 5)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.save_map_srv = self.create_service(Trigger, '/slam/save_map', self.on_save_map)
        self.load_map_srv = self.create_service(Trigger, '/slam/load_map', self.on_load_map)

        if self.publish_landmark_markers:
            self.create_timer(0.25, self.publish_landmark_markers_msg)
        self.create_timer(self.occupancy_publish_period_sec, self.publish_occupancy_grid)

        if self.map_file_path:
            path = os.path.expanduser(self.map_file_path)
            if os.path.isfile(path):
                if self.load_map_from_file(path):
                    self.get_logger().info(f'loaded EKF-SLAM map from {path}')
                else:
                    self.get_logger().warn(f'failed to load EKF-SLAM map from {path}')

        self.get_logger().info('slam node started (standard EKF-SLAM with cone landmark updates)')

    def state_dim(self) -> int:
        return self.x.shape[0]

    def landmark_count(self) -> int:
        return max(0, (self.state_dim() - self.base_dim) // 2)

    def landmark_slice(self, landmark_index: int) -> slice:
        start = self.base_dim + 2 * landmark_index
        return slice(start, start + 2)

    def on_state_str(self, msg: String) -> None:
        state = msg.data
        driving = ('AS:DRIVING' in state) or ('AMI:MANUAL' in state)
        if ('AS:OFF' in state or 'AS:READY' in state) and self.filter_ready:
            self.reset_filter()
        self.motion_enabled = driving

    def on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = stamp
            self.last_update_time = stamp
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

        if not self.motion_enabled:
            omega_z = 0.0
            accel_x = 0.0

        px = float(self.x[0, 0]) + v * math.cos(yaw) * dt + 0.5 * accel_x * math.cos(yaw) * dt * dt
        py = float(self.x[1, 0]) + v * math.sin(yaw) * dt + 0.5 * accel_x * math.sin(yaw) * dt * dt
        yaw_new = self.normalize_angle(yaw + omega_z * dt)
        v_new = v + accel_x * dt

        self.x[0, 0] = px
        self.x[1, 0] = py
        self.x[2, 0] = yaw_new
        self.x[3, 0] = v_new

        n = self.state_dim()
        F = np.eye(n)
        F[0, 2] = -v * math.sin(yaw) * dt - 0.5 * accel_x * math.sin(yaw) * dt * dt
        F[0, 3] = math.cos(yaw) * dt
        F[1, 2] = v * math.cos(yaw) * dt + 0.5 * accel_x * math.cos(yaw) * dt * dt
        F[1, 3] = math.sin(yaw) * dt
        F[2, 4] = -dt

        q_px = (0.5 * self.process_accel_noise * dt * dt) ** 2
        q_py = q_px
        q_yaw = (self.process_gyro_noise * dt) ** 2
        q_v = (self.process_accel_noise * dt) ** 2
        q_bg = (self.process_gyro_bias_rw * math.sqrt(dt)) ** 2
        Q = np.zeros((n, n), dtype=float)
        Q[0, 0] = q_px
        Q[1, 1] = q_py
        Q[2, 2] = q_yaw
        Q[3, 3] = q_v
        Q[4, 4] = q_bg

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

        self.last_update_time = stamp
        self.publish_outputs(stamp)

    def on_wheel_stamped(self, msg: WheelSpeedsStamped) -> None:
        self.on_wheel(msg.speeds)

    def on_wheel(self, msg: WheelSpeeds) -> None:
        rear_rpm = 0.5 * (abs(float(msg.lb_speed)) + abs(float(msg.rb_speed)))
        z = (rear_rpm / 60.0) * (2.0 * math.pi * self.wheel_radius_m)
        self.ekf_update_speed(z)

    def on_cones(self, msg: ConeArrayWithCovariance) -> None:
        if not self.filter_ready or not self.motion_enabled:
            return

        observations = []
        for cone in msg.blue_cones:
            observations.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.yellow_cones:
            observations.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.orange_cones:
            observations.append((float(cone.point.x), float(cone.point.y)))
        for cone in msg.big_orange_cones:
            observations.append((float(cone.point.x), float(cone.point.y)))

        if not observations:
            return

        R = np.diag([self.cone_meas_noise_x ** 2, self.cone_meas_noise_y ** 2])
        processed = 0
        matched_landmark_indices = []

        for zx, zy in observations:
            if processed >= self.max_observations_per_scan:
                break

            if not math.isfinite(zx) or not math.isfinite(zy):
                continue
            if math.hypot(zx, zy) > self.obs_max_range_m:
                continue

            lm_index = self.associate_landmark(zx, zy, R)
            if lm_index is None:
                if self.landmark_count() < self.max_landmarks:
                    self.augment_landmark(zx, zy, R)
            else:
                self.ekf_update_landmark(lm_index, zx, zy, R)
                matched_landmark_indices.append(lm_index)

            self.integrate_occupancy_observation(zx, zy)

            processed += 1

        if self.enable_loop_closure:
            self.update_loop_closure(matched_landmark_indices)

    def ekf_update_speed(self, speed_measurement: float) -> None:
        if not self.filter_ready:
            return

        n = self.state_dim()
        H = np.zeros((1, n), dtype=float)
        H[0, 3] = 1.0
        R = np.array([[self.wheel_speed_noise ** 2]], dtype=float)

        z = np.array([[speed_measurement]], dtype=float)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(n)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def associate_landmark(self, zx: float, zy: float, R: np.ndarray) -> Optional[int]:
        landmark_count = self.landmark_count()
        if landmark_count == 0:
            return None

        best_idx = None
        best_d2 = float('inf')
        z = np.array([[zx], [zy]], dtype=float)

        for idx in range(landmark_count):
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

        odom.pose.covariance[0] = float(self.P[0, 0])
        odom.pose.covariance[1] = float(self.P[0, 1])
        odom.pose.covariance[6] = float(self.P[1, 0])
        odom.pose.covariance[7] = float(self.P[1, 1])
        odom.pose.covariance[35] = float(self.P[2, 2])
        odom.twist.covariance[0] = float(self.P[3, 3])

        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path_msg.header.stamp = odom.header.stamp
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.path_max_length:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_length:]
        self.path_pub.publish(self.path_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = odom.header
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_landmark_markers_msg(self) -> None:
        if not self.publish_landmark_markers:
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        count = self.landmark_count()

        for idx in range(count):
            sl = self.landmark_slice(idx)
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = 'slam_landmarks'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(self.x[sl.start, 0])
            marker.pose.position.y = float(self.x[sl.start + 1, 0])
            marker.pose.position.z = 0.0
            marker.scale.x = self.landmark_marker_scale
            marker.scale.y = self.landmark_marker_scale
            marker.scale.z = self.landmark_marker_scale
            marker.color.a = 0.95
            marker.color.r = 1.0
            marker.color.g = 0.68
            marker.color.b = 0.1
            marker_array.markers.append(marker)

        if self.last_landmark_marker_count > count:
            for idx in range(count, self.last_landmark_marker_count):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = stamp
                marker.ns = 'slam_landmarks'
                marker.id = idx
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)

        self.last_landmark_marker_count = count
        self.landmark_pub.publish(marker_array)

    def publish_occupancy_grid(self) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = float(self.occupancy_resolution_m)
        grid.info.width = int(self.occupancy_width_cells)
        grid.info.height = int(self.occupancy_height_cells)
        grid.info.origin.position.x = float(self.occupancy_origin_x)
        grid.info.origin.position.y = float(self.occupancy_origin_y)
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        p = 1.0 / (1.0 + np.exp(-self.occupancy_logodds))
        occ = np.rint(p * 100.0).astype(np.int16)
        occ = np.clip(occ, 0, 100).astype(np.int8)
        grid.data = occ.flatten().tolist()
        self.occupancy_pub.publish(grid)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.occupancy_origin_x) / self.occupancy_resolution_m)
        gy = int((y - self.occupancy_origin_y) / self.occupancy_resolution_m)
        return gx, gy

    def in_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.occupancy_width_cells and 0 <= gy < self.occupancy_height_cells

    def bresenham(self, x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def integrate_occupancy_observation(self, zx: float, zy: float) -> None:
        yaw = float(self.x[2, 0])
        c = math.cos(yaw)
        s = math.sin(yaw)
        px = float(self.x[0, 0])
        py = float(self.x[1, 0])

        wx = px + c * zx - s * zy
        wy = py + s * zx + c * zy

        gx0, gy0 = self.world_to_grid(px, py)
        gx1, gy1 = self.world_to_grid(wx, wy)
        if not self.in_grid(gx0, gy0) or not self.in_grid(gx1, gy1):
            return

        ray = list(self.bresenham(gx0, gy0, gx1, gy1))
        if not ray:
            return

        for cx, cy in ray[:-1]:
            if self.in_grid(cx, cy):
                self.occupancy_logodds[cy, cx] += self.occupancy_logodds_miss

        end_x, end_y = ray[-1]
        if self.in_grid(end_x, end_y):
            self.occupancy_logodds[end_y, end_x] += self.occupancy_logodds_hit

        np.clip(
            self.occupancy_logodds,
            self.occupancy_logodds_min,
            self.occupancy_logodds_max,
            out=self.occupancy_logodds,
        )

    def get_nearby_landmark_ids(self, x: float, y: float, radius: float) -> set:
        ids = set()
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

    def update_loop_closure(self, matched_landmark_indices: list) -> None:
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

    def on_save_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self.map_file_path:
            response.success = False
            response.message = 'map_file_path parameter is empty'
            return response

        path = os.path.expanduser(self.map_file_path)
        ok = self.save_map_to_file(path)
        response.success = ok
        response.message = f'map saved to {path}' if ok else f'failed to save map to {path}'
        return response

    def on_load_map(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self.map_file_path:
            response.success = False
            response.message = 'map_file_path parameter is empty'
            return response

        path = os.path.expanduser(self.map_file_path)
        ok = self.load_map_from_file(path)
        response.success = ok
        response.message = f'map loaded from {path}' if ok else f'failed to load map from {path}'
        return response

    def save_map_to_file(self, file_path: str) -> bool:
        try:
            os.makedirs(os.path.dirname(file_path), exist_ok=True) if os.path.dirname(file_path) else None
            landmarks = []
            for idx in range(self.landmark_count()):
                sl = self.landmark_slice(idx)
                landmarks.append({
                    'x': float(self.x[sl.start, 0]),
                    'y': float(self.x[sl.start + 1, 0]),
                })

            payload = {
                'version': 1,
                'frame': self.map_frame,
                'landmarks': landmarks,
                'occupancy': {
                    'resolution': self.occupancy_resolution_m,
                    'width': self.occupancy_width_cells,
                    'height': self.occupancy_height_cells,
                    'origin_x': self.occupancy_origin_x,
                    'origin_y': self.occupancy_origin_y,
                    'logodds': self.occupancy_logodds.tolist(),
                },
            }
            with open(file_path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2)
            return True
        except Exception as exc:
            self.get_logger().error(f'save_map_to_file error: {exc}')
            return False

    def load_map_from_file(self, file_path: str) -> bool:
        try:
            with open(file_path, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)

            landmarks = payload.get('landmarks', [])
            if not isinstance(landmarks, list):
                return False

            x_new = np.zeros((self.base_dim + 2 * len(landmarks), 1), dtype=float)
            x_new[:self.base_dim, 0] = self.x[:self.base_dim, 0]
            P_new = np.zeros((x_new.shape[0], x_new.shape[0]), dtype=float)
            P_new[:self.base_dim, :self.base_dim] = self.P[:self.base_dim, :self.base_dim]

            lm_var = self.loaded_landmark_std ** 2
            for idx, lm in enumerate(landmarks):
                start = self.base_dim + 2 * idx
                lx = float(lm.get('x', 0.0))
                ly = float(lm.get('y', 0.0))
                x_new[start, 0] = lx
                x_new[start + 1, 0] = ly
                P_new[start, start] = lm_var
                P_new[start + 1, start + 1] = lm_var

            self.x = x_new
            self.P = 0.5 * (P_new + P_new.T)

            occ = payload.get('occupancy', None)
            if isinstance(occ, dict):
                h = int(occ.get('height', self.occupancy_height_cells))
                w = int(occ.get('width', self.occupancy_width_cells))
                logodds = occ.get('logodds', None)
                if isinstance(logodds, list) and h == self.occupancy_height_cells and w == self.occupancy_width_cells:
                    arr = np.array(logodds, dtype=float)
                    if arr.shape == (h, w):
                        self.occupancy_logodds[:, :] = arr
            return True
        except Exception as exc:
            self.get_logger().error(f'load_map_from_file error: {exc}')
            return False

    def reset_filter(self) -> None:
        self.x = np.zeros((self.base_dim, 1), dtype=float)
        self.P = self.base_P0.copy()
        self.path_msg.poses.clear()
        self.keyframes.clear()
        self.last_loop_time = -1e9
        self.occupancy_logodds[:, :] = 0.0
        self.last_imu_time = None
        self.last_update_time = None
        self.filter_ready = False

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
    node = Slam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.auto_save_map_on_shutdown and node.map_file_path:
            path = os.path.expanduser(node.map_file_path)
            node.save_map_to_file(path)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
